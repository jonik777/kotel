#include "reEvents.h"
#include "sensors.h"
#include "strings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include "project_config.h"
#include "def_consts.h"
#include "esp_timer.h"
#include "rLog.h"
#include "rTypes.h"
#include "rStrings.h"
#include "reStates.h"
#include "reMqtt.h"
#include "reParams.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "reWiFi.h"
#include "reRangeMonitor.h"
#include "reLoadCtrl.h"
#if CONFIG_TELEGRAM_ENABLE
#include "reTgSend.h"
#endif // CONFIG_TELEGRAM_ENABLE
#if CONFIG_DATASEND_ENABLE
#include "reDataSend.h"
#endif // CONFIG_DATASEND_ENABLE
#if defined(CONFIG_ELTARIFFS_ENABLED) && CONFIG_ELTARIFFS_ENABLED
#include "reElTariffs.h"
#endif // CONFIG_ELTARIFFS_ENABLED
#include "lcd_display.h"

// Определения объектов
// sensorOutdoor: DS18B20 на GPIO 0 (заменён с DHT22/AM2302)
DS18x20 sensorOutdoor(1);
BME280 sensorIndoor(2);
DS18x20 sensorBoiler(3);
reLCD lcd(CONFIG_LCD_I2C_BUS, CONFIG_LCD_I2C_ADDRESS, CONFIG_LCD_I2C_TYPE);

static const char* logTAG = "SENS";
static const char* sensorsTaskName = "sensors";
static TaskHandle_t _sensorsTask;
static bool _sensorsNeedStore = false;
// Флаг валидности данных датчика улицы: false при мусорных/пустых фреймах DHT22.
// Используется в sensorsBoilerControl() и при передаче на LCD.
static bool _outdoorDataValid = false;

// Handle параметра уставки температуры — нужен для публикации в MQTT при режиме интерполяции
static paramsEntryHandle_t paramThermostatTemp = nullptr;

// Параметры регулирования температуры в доме
float thermostatInternalTemp = 22.0;
float thermostatInternalHyst = 5.0;
timespan_t thermostatTimespan = 15000800U;
thermostat_mode_t thermostatMode = THERMOSTAT_TIME_AND_TEMP;
bool thermostatNotify = true;

// Массив точек интерполяции (по умолчанию 6 фиксированных точек)
interpolation_point_t interpolation_points[MAX_INTERPOLATION_POINTS] = {
    {-30.0f, 70.0f},  // -30°C -> 70°C
    {-20.0f, 65.0f},  // -20°C -> 65°C
    {-10.0f, 58.0f},  // -10°C -> 58°C
    {  0.0f, 42.0f},  //   0°C -> 42°C
    { 10.0f, 35.0f},  //  10°C -> 35°C
    { 20.0f, 25.0f}   //  20°C -> 25°C
};
uint8_t interpolation_points_count = CONTROL_THERMOSTAT_INTERP_POINTS;

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Термостат ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

//отправка состояния (on/off) бойлера в телеграмм
static void boilerStateChange(rLoadController *ctrl, bool state, time_t duration)
{
  /*if (thermostatNotify) {
    if (state) {
      tgSend(CONTROL_THERMOSTAT_NOTIFY_KIND, CONTROL_THERMOSTAT_NOTIFY_PRIORITY, CONTROL_THERMOSTAT_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_THERMOSTAT_NOTIFY_ON);
    } else {
      tgSend(CONTROL_THERMOSTAT_NOTIFY_KIND, CONTROL_THERMOSTAT_NOTIFY_PRIORITY, CONTROL_THERMOSTAT_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_THERMOSTAT_NOTIFY_OFF);
    };
  };  */
}

bool boilerMqttPublish(rLoadController *ctrl, char* topic, char* payload, bool free_topic, bool free_payload)
{
  return mqttPublish(topic, payload, CONTROL_THERMOSTAT_QOS, CONTROL_THERMOSTAT_RETAINED, free_topic, free_payload);
}

rLoadGpioController lcBoiler(
    CONFIG_GPIO_RELAY_BOILER,           // GPIO, к которому подключено реле
    0x01,                               // Реле включается высоким уровнем на выходе
    false,                              // Таймер не нужен
    CONTROL_THERMOSTAT_BOILER_KEY,      // Ключ, по которому будет хранится статистика в NVS space
    nullptr, nullptr, TI_MILLISECONDS,  // Указатели на параметры периодического управления нагрузкой, не нужны
    nullptr, nullptr,                   // Указатели на callbacks, которые будут вызываны перед и после изменения состояния выхода
    boilerStateChange,                  // Функция обратного вызова, которая будет вызывана при изменении состояния нагрузки
    boilerMqttPublish                   // Функция обратного вызова, которая будет вызывана при необходимости отправить данные на MQTT брокер
  );

void sensorsInitRelays()
{
  #if defined(CONFIG_ELTARIFFS_ENABLED) && CONFIG_ELTARIFFS_ENABLED
  lcBoiler.setPeriodStartDay(elTariffsGetReportDayAddress());
  #endif // CONFIG_ELTARIFFS_ENABLED
  lcBoiler.countersNvsRestore();
  lcBoiler.loadInit(false);
}

bool sensorsBoilerTempCheck()
{
  // Получаем текущее состояние нагрузки
  bool oldState = lcBoiler.getState();

  // Получаем текущую температуру с сенсора (она хранится во втором "хранилище") без физического чтения данных с шины
  float tempBoiler = NAN;
  if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
    tempBoiler = sensorBoiler.getValue(false).filteredValue;
  };

  // Если удалось считать температуру, проверяем её в зависимости от текущего состояния нагрузки
  if (!isnan(tempBoiler)) {
    if (oldState) {
      // Сейчас котел включен. Выключить мы его должны, когда температура достигнет порогового + 1/2 гистерезиса
      return tempBoiler < (thermostatInternalTemp + 0 * thermostatInternalHyst);
    } else {
      // Сейчас котел выключен. Включить мы его должны, когда температура снизится до порогового - 1/2 гистерезиса
      return tempBoiler < (thermostatInternalTemp - 1 * thermostatInternalHyst);
    };
  };
  return false;
}

void sensorsBoilerControl()
{
  bool newState;

  // Котел выключен всегда
  if (thermostatMode == THERMOSTAT_OFF) {
    newState = false;
  } 
  // Котел включен всегда (без учета расписания и температуры)
  else if (thermostatMode == THERMOSTAT_ON) {
    newState = true;
  } 
  // Только управление по расписанию (без учета температуры)
  else if (thermostatMode == THERMOSTAT_TIME) {
    newState = checkTimespanNowEx(thermostatTimespan, true);
  } 
  // Только управление по температуре (без учета расписания)
  else if (thermostatMode == THERMOSTAT_TEMP) {
    newState = sensorsBoilerTempCheck();
  } 
  // Управление по расписанию и температуре одновременно
  else if (thermostatMode == THERMOSTAT_TIME_AND_TEMP) {
    newState = checkTimespanNowEx(thermostatTimespan, true) && sensorsBoilerTempCheck();
  }
  // Управление по интерполяции
  else if (thermostatMode == THERMOSTAT_INTERPOLATION) {
    if (interpolation_points_count < 2) {
      newState = false; // Недостаточно точек для интерполяции
    } else {
      float outdoor_temp = NAN;
      float boiler_temp = NAN;
      
      // Получаем температуру с датчика наружного воздуха
      if (sensorOutdoor.getStatus() == SENSOR_STATUS_OK && _outdoorDataValid) {
        float v = sensorOutdoor.getValue(false).filteredValue;
        // Дополнительная защита: медиана могла ещё не "промыться" после выброса
        if ((v >= SENSOR_OUTDOOR_TEMP_MIN) && (v <= SENSOR_OUTDOOR_TEMP_MAX)) {
          outdoor_temp = v;
        } else {
          rlog_w(logTAG, "Outdoor filtered value out of range: %.2f C, treating as unavailable", v);
        }
      }
      
      // Получаем температуру с датчика бойлера
      if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
        boiler_temp = sensorBoiler.getValue(false).filteredValue;
      }
      
      if (isnan(outdoor_temp) || isnan(boiler_temp)) {
        // Нет данных с датчика наружной температуры — безопасный режим:
        // используем фиксированную уставку как при сильном морозе, но не максимум.
        // Котёл работает по температуре теплоносителя с безопасной уставкой.
        if (!isnan(boiler_temp)) {
          float safe_target = interpolation_points[0].boiler_temp; // самая холодная точка таблицы
          rlog_w(logTAG, "Outdoor sensor unavailable in INTERPOLATION mode, using safe fallback target: %.1f C", safe_target);
          thermostatInternalTemp = safe_target;
          if (paramThermostatTemp) paramsMqttPublish(paramThermostatTemp, true);
          if (lcBoiler.getState()) {
            newState = boiler_temp < (safe_target + 0 * thermostatInternalHyst);
          } else {
            newState = boiler_temp < (safe_target - 1 * thermostatInternalHyst);
          }
        } else {
          newState = false; // Нет данных вообще — котёл не трогаем
        }
      } else {
        // Рассчитываем целевую температуру бойлера
        float target_temp = calculateInterpolatedTemp(outdoor_temp, interpolation_points, interpolation_points_count);
        
        // Обновляем уставку термостата для отображения
        thermostatInternalTemp = target_temp;
        
        // Публикуем новое значение уставки в MQTT
        if (paramThermostatTemp) {
          paramsMqttPublish(paramThermostatTemp, true);
        };
        
        // Включаем котел, если температура ниже целевой с учетом гистерезиса
        if (lcBoiler.getState()) {
          // Сейчас котел включен. Выключить мы его должны, когда температура достигнет порогового + 1/2 гистерезиса
          newState = boiler_temp < (target_temp + 0 * thermostatInternalHyst);
        } else {
          // Сейчас котел выключен. Включить мы его должны, когда температура снизится до порогового - 1/2 гистерезиса
          newState = boiler_temp < (target_temp - 1 * thermostatInternalHyst);
        }
      }
    }
  }
  // Защита от ошибки программиста
  else {
    newState = false;
  };

  // Применяем новое состояние
  lcBoiler.loadSetState(newState, false, true);
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Сенсоры ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

paramsGroupHandle_t pgSensors;
paramsGroupHandle_t pgIntervals;
paramsGroupHandle_t pgTempMonitor;

static bool sensorsPublish(rSensor *sensor, char* topic, char* payload, const bool free_topic, const bool free_payload)
{
  return mqttPublish(topic, payload, CONFIG_MQTT_SENSORS_QOS, CONFIG_MQTT_SENSORS_RETAINED, free_topic, free_payload);
}

static bool monitorPublish(reRangeMonitor *monitor, char* topic, char* payload, bool free_topic, bool free_payload)
{
  return mqttPublish(topic, payload, CONTROL_TEMP_QOS, CONTROL_TEMP_RETAINED, free_topic, free_payload);
}

static void sensorsMqttTopicsCreate(bool primary)
{
  sensorOutdoor.topicsCreate(primary);
  sensorIndoor.topicsCreate(primary);
  if (tempMonitorIndoor.mqttTopicCreate(primary, CONTROL_TEMP_LOCAL, CONTROL_TEMP_GROUP_TOPIC, CONTROL_TEMP_INDOOR_TOPIC, nullptr)) {
    rlog_i(logTAG, "Generated topic for indoor temperture control: [ %s ]", tempMonitorIndoor.mqttTopicGet());
  };
  sensorBoiler.topicsCreate(primary);
  if (tempMonitorBoiler.mqttTopicCreate(primary, CONTROL_TEMP_LOCAL, CONTROL_TEMP_GROUP_TOPIC, CONTROL_TEMP_BOILER_TOPIC, nullptr)) {
    rlog_i(logTAG, "Generated topic for boiler temperture control: [ %s ]", tempMonitorBoiler.mqttTopicGet());
  };
  lcBoiler.mqttTopicCreate(primary, CONTROL_THERMOSTAT_LOCAL, CONTROL_THERMOSTAT_BOILER_TOPIC, nullptr, nullptr);
}

static void sensorsMqttTopicsFree()
{
  sensorOutdoor.topicsFree();
  sensorIndoor.topicsFree();
  tempMonitorIndoor.mqttTopicFree();
  sensorBoiler.topicsFree();
  tempMonitorBoiler.mqttTopicFree();
  lcBoiler.mqttTopicFree();
  rlog_d(logTAG, "Topics for temperture control has been scrapped");
}


static void monitorNotifyIndoor(reRangeMonitor *monitor, range_monitor_status_t status, bool notify, float value, float min, float max)
{
  if (notify) {
    if (status == TMS_NORMAL) {
      tgSend(CONTROL_TEMP_INDOOR_NOTIFY_KIND, CONTROL_TEMP_INDOOR_NOTIFY_PRIORITY, CONTROL_TEMP_INDOOR_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_TEMP_INDOOR_NOTIFY_NORMAL, value);
    } else if (status == TMS_TOO_LOW) {
      tgSend(CONTROL_TEMP_INDOOR_NOTIFY_KIND, CONTROL_TEMP_INDOOR_NOTIFY_PRIORITY, CONTROL_TEMP_INDOOR_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_TEMP_INDOOR_NOTIFY_TOO_LOW, value);
    } else if (status == TMS_TOO_HIGH) {
      tgSend(CONTROL_TEMP_INDOOR_NOTIFY_KIND, CONTROL_TEMP_INDOOR_NOTIFY_PRIORITY, CONTROL_TEMP_INDOOR_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_TEMP_INDOOR_NOTIFY_TOO_HIGH, value);
    } 
  }
}

static void monitorNotifyBoiler(reRangeMonitor *monitor, range_monitor_status_t status, bool notify, float value, float min, float max)
{
  if (notify) {
    if (status == TMS_NORMAL) {
      tgSend(CONTROL_TEMP_BOILER_NOTIFY_KIND, CONTROL_TEMP_BOILER_NOTIFY_PRIORITY, CONTROL_TEMP_BOILER_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_TEMP_BOILER_NOTIFY_NORMAL, value);
    } else if (status == TMS_TOO_LOW) {
      tgSend(CONTROL_TEMP_BOILER_NOTIFY_KIND, CONTROL_TEMP_BOILER_NOTIFY_PRIORITY, CONTROL_TEMP_BOILER_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_TEMP_BOILER_NOTIFY_TOO_LOW, value);
    } else if (status == TMS_TOO_HIGH) {
      tgSend(CONTROL_TEMP_BOILER_NOTIFY_KIND, CONTROL_TEMP_BOILER_NOTIFY_PRIORITY, CONTROL_TEMP_BOILER_NOTIFY_ALARM, CONFIG_TELEGRAM_DEVICE, 
        CONTROL_TEMP_BOILER_NOTIFY_TOO_HIGH, value);
    } 
  }
}

static void sensorsStoreData()
{
  rlog_i(logTAG, "Store sensors data");

  sensorOutdoor.nvsStoreExtremums(SENSOR_OUTDOOR_KEY);
  sensorIndoor.nvsStoreExtremums(SENSOR_INDOOR_KEY);
  sensorBoiler.nvsStoreExtremums(SENSOR_BOILER_KEY);

  tempMonitorIndoor.nvsStore(CONTROL_TEMP_INDOOR_KEY);
  tempMonitorBoiler.nvsStore(CONTROL_TEMP_BOILER_KEY);

  lcBoiler.countersNvsStore();
}

static void sensorsInitParameters()
{
  // Группы параметров
  pgSensors = paramsRegisterGroup(nullptr, 
    CONFIG_SENSOR_PGROUP_ROOT_KEY, CONFIG_SENSOR_PGROUP_ROOT_TOPIC, CONFIG_SENSOR_PGROUP_ROOT_FRIENDLY);
  pgIntervals = paramsRegisterGroup(pgSensors, 
    CONFIG_SENSOR_PGROUP_INTERVALS_KEY, CONFIG_SENSOR_PGROUP_INTERVALS_TOPIC, CONFIG_SENSOR_PGROUP_INTERVALS_FRIENDLY);
  pgTempMonitor = paramsRegisterGroup(nullptr, 
    CONTROL_TEMP_GROUP_KEY, CONTROL_TEMP_GROUP_TOPIC, CONTROL_TEMP_GROUP_FRIENDLY);

  paramsGroupHandle_t pgThermostat = paramsRegisterGroup(nullptr, 
    CONTROL_THERMOSTAT_GROUP_KEY, CONTROL_THERMOSTAT_GROUP_TOPIC, CONTROL_THERMOSTAT_GROUP_FRIENDLY);

  // Период публикации данных с сенсоров на MQTT
  if (pgIntervals) {
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgIntervals,
      CONFIG_SENSOR_PARAM_INTERVAL_MQTT_KEY, CONFIG_SENSOR_PARAM_INTERVAL_MQTT_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&iMqttPubInterval);

    #if CONFIG_OPENMON_ENABLE
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgIntervals,
        CONFIG_SENSOR_PARAM_INTERVAL_OPENMON_KEY, CONFIG_SENSOR_PARAM_INTERVAL_OPENMON_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&iOpenMonInterval);
    #endif // CONFIG_OPENMON_ENABLE

    #if CONFIG_NARODMON_ENABLE
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgIntervals,
        CONFIG_SENSOR_PARAM_INTERVAL_NARODMON_KEY, CONFIG_SENSOR_PARAM_INTERVAL_NARODMON_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&iNarodMonInterval);
    #endif // CONFIG_NARODMON_ENABLE

    #if CONFIG_THINGSPEAK_ENABLE
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgIntervals,
        CONFIG_SENSOR_PARAM_INTERVAL_THINGSPEAK_KEY, CONFIG_SENSOR_PARAM_INTERVAL_THINGSPEAK_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&iThingSpeakInterval);
    #endif // CONFIG_THINGSPEAK_ENABLE
  };

  // Параметры термостата
  if (pgThermostat) {
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgThermostat,
      CONTROL_THERMOSTAT_PARAM_MODE_KEY, CONTROL_THERMOSTAT_PARAM_MODE_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&thermostatMode);
    paramThermostatTemp = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, pgThermostat,
      CONTROL_THERMOSTAT_PARAM_TEMP_KEY, CONTROL_THERMOSTAT_PARAM_TEMP_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&thermostatInternalTemp);
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, pgThermostat,
      CONTROL_THERMOSTAT_PARAM_HYST_KEY, CONTROL_THERMOSTAT_PARAM_HYST_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&thermostatInternalHyst);
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_TIMESPAN, nullptr, pgThermostat,
      CONTROL_THERMOSTAT_PARAM_TIME_KEY, CONTROL_THERMOSTAT_PARAM_TIME_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&thermostatTimespan);
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgThermostat,
      CONTROL_THERMOSTAT_PARAM_NOTIFY_KEY, CONTROL_THERMOSTAT_PARAM_NOTIFY_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&thermostatNotify);

    // Точки интерполяции термостата
    paramsGroupHandle_t pgThermostatInterp = paramsRegisterGroup(
      pgThermostat,
      CONTROL_THERMOSTAT_INTERP_GROUP_KEY,
      CONTROL_THERMOSTAT_INTERP_GROUP_TOPIC,
      CONTROL_THERMOSTAT_INTERP_GROUP_FRIENDLY
    );

    if (pgThermostatInterp) {
      static const char* interpOutKeys[CONTROL_THERMOSTAT_INTERP_POINTS] = {
        "p1_out", "p2_out", "p3_out", "p4_out", "p5_out", "p6_out"
      };
      static const char* interpOutFriendly[CONTROL_THERMOSTAT_INTERP_POINTS] = {
        "Улица P1", "Улица P2", "Улица P3",
        "Улица P4", "Улица P5", "Улица P6"
      };
      static const char* interpBoilerKeys[CONTROL_THERMOSTAT_INTERP_POINTS] = {
        "p1_bo", "p2_bo", "p3_bo", "p4_bo", "p5_bo", "p6_bo"
      };
      static const char* interpBoilerFriendly[CONTROL_THERMOSTAT_INTERP_POINTS] = {
        "Котёл P1", "Котёл P2", "Котёл P3",
        "Котёл P4", "Котёл P5", "Котёл P6"
      };

      for (int i = 0; i < CONTROL_THERMOSTAT_INTERP_POINTS; i++) {
        paramsRegisterValue(
          OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, pgThermostatInterp,
          interpOutKeys[i], interpOutFriendly[i],
          CONFIG_MQTT_PARAMS_QOS, (void*)&interpolation_points[i].outdoor_temp
        );
        paramsRegisterValue(
          OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, pgThermostatInterp,
          interpBoilerKeys[i], interpBoilerFriendly[i],
          CONFIG_MQTT_PARAMS_QOS, (void*)&interpolation_points[i].boiler_temp
        );
      }
    }
  };
}

static void sensorsInitSensors()
{
  // Улица — DS18B20 на GPIO CONFIG_GPIO_AM2320 (GPIO 0, бывший DHT22)
  // DS18x20 — один канал, только температура. Влажность улицы недоступна.
  static rTemperatureItem siOutdoorTemp(nullptr, CONFIG_SENSOR_TEMP_NAME, CONFIG_FORMAT_TEMP_UNIT,
    SENSOR_OUTDOOR_FILTER_MODE, SENSOR_OUTDOOR_FILTER_SIZE, 
    CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  sensorOutdoor.initExtItems(SENSOR_OUTDOOR_NAME, SENSOR_OUTDOOR_TOPIC, false,
    (gpio_num_t)CONFIG_GPIO_AM2320, ONEWIRE_NONE, 1, DS18x20_RESOLUTION_12_BIT, true,
    &siOutdoorTemp,
    3000, SENSOR_OUTDOOR_ERRORS_LIMIT, nullptr, sensorsPublish);
  sensorOutdoor.registerParameters(pgSensors, SENSOR_OUTDOOR_KEY, SENSOR_OUTDOOR_TOPIC, SENSOR_OUTDOOR_NAME);
  sensorOutdoor.nvsRestoreExtremums(SENSOR_OUTDOOR_KEY);

  // Комната
  static rPressureItem siIndoorPress(nullptr, CONFIG_SENSOR_PRESSURE_NAME, CONFIG_FORMAT_PRESSURE_UNIT,
    SENSOR_INDOOR_FILTER_MODE, SENSOR_INDOOR_FILTER_SIZE, 
    CONFIG_FORMAT_PRESSURE_VALUE, CONFIG_FORMAT_PRESSURE_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  static rTemperatureItem siIndoorTemp(nullptr, CONFIG_SENSOR_TEMP_NAME, CONFIG_FORMAT_TEMP_UNIT,
    SENSOR_INDOOR_FILTER_MODE, SENSOR_INDOOR_FILTER_SIZE, 
    CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  static rSensorItem siIndoorHum(nullptr, CONFIG_SENSOR_HUMIDITY_NAME, 
    SENSOR_INDOOR_FILTER_MODE, SENSOR_INDOOR_FILTER_SIZE, 
    CONFIG_FORMAT_HUMIDITY_VALUE, CONFIG_FORMAT_HUMIDITY_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  sensorIndoor.initExtItems(SENSOR_INDOOR_NAME, SENSOR_INDOOR_TOPIC, false,
    SENSOR_INDOOR_BUS, SENSOR_INDOOR_ADDRESS, 
    BME280_MODE_FORCED, BME280_STANDBY_1000ms, BME280_FLT_NONE, BME280_OSM_X4, BME280_OSM_X4, BME280_OSM_X4,
    &siIndoorPress, &siIndoorTemp, &siIndoorHum, 
    3000, SENSOR_INDOOR_ERRORS_LIMIT, nullptr, sensorsPublish);
  sensorIndoor.registerParameters(pgSensors, SENSOR_INDOOR_KEY, SENSOR_INDOOR_TOPIC, SENSOR_INDOOR_NAME);
  sensorIndoor.nvsRestoreExtremums(SENSOR_INDOOR_KEY);
  tempMonitorIndoor.nvsRestore(CONTROL_TEMP_INDOOR_KEY);
  tempMonitorIndoor.setStatusCallback(monitorNotifyIndoor);
  tempMonitorIndoor.mqttSetCallback(monitorPublish);
  tempMonitorIndoor.paramsRegister(pgTempMonitor, CONTROL_TEMP_INDOOR_KEY, CONTROL_TEMP_INDOOR_TOPIC, CONTROL_TEMP_INDOOR_FRIENDLY);

  // Теплоноситель
  static rTemperatureItem siBoilerTemp(nullptr, CONFIG_SENSOR_TEMP_NAME, CONFIG_FORMAT_TEMP_UNIT,
    SENSOR_BOILER_FILTER_MODE, SENSOR_BOILER_FILTER_SIZE, 
    CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  sensorBoiler.initExtItems(SENSOR_BOILER_NAME, SENSOR_BOILER_TOPIC, false,
    (gpio_num_t)CONFIG_GPIO_DS18B20, ONEWIRE_NONE, 1, DS18x20_RESOLUTION_12_BIT, true, 
    &siBoilerTemp,
    3000, SENSOR_BOILER_ERRORS_LIMIT, nullptr, sensorsPublish);
  sensorBoiler.registerParameters(pgSensors, SENSOR_BOILER_KEY, SENSOR_BOILER_TOPIC, SENSOR_BOILER_NAME);
  sensorBoiler.nvsRestoreExtremums(SENSOR_BOILER_KEY);
  tempMonitorBoiler.nvsRestore(CONTROL_TEMP_BOILER_KEY);
  tempMonitorBoiler.setStatusCallback(monitorNotifyBoiler);
  tempMonitorBoiler.mqttSetCallback(monitorPublish);
  tempMonitorBoiler.paramsRegister(pgTempMonitor, CONTROL_TEMP_BOILER_KEY, CONTROL_TEMP_BOILER_TOPIC, CONTROL_TEMP_BOILER_FRIENDLY);

  espRegisterShutdownHandler(sensorsStoreData); // #2
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- MQTT ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void sensorsMqttEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  // MQTT connected
  if (event_id == RE_MQTT_CONNECTED) {
    re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
    sensorsMqttTopicsCreate(data->primary);
  } 
  // MQTT disconnected
  else if ((event_id == RE_MQTT_CONN_LOST) || (event_id == RE_MQTT_CONN_FAILED)) {
    sensorsMqttTopicsFree();
  }
}

static void sensorsTimeEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if (event_id == RE_TIME_START_OF_DAY) {
    _sensorsNeedStore = true;
  };
  lcBoiler.countersTimeEventHandler(event_id, event_data);
}

static void sensorsResetExtremumsSensor(rSensor* sensor, const char* sensor_name, uint8_t mode) 
{ 
  if (mode == 0) {
    sensor->resetExtremumsTotal();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_TOTAL_DEV, sensor_name);
    #endif // CONFIG_TELEGRAM_ENABLE
  } else if (mode == 1) {
    sensor->resetExtremumsDaily();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_DAILY_DEV, sensor_name);
    #endif // CONFIG_TELEGRAM_ENABLE
  } else if (mode == 2) {
    sensor->resetExtremumsWeekly();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_WEEKLY_DEV, sensor_name);
    #endif // CONFIG_TELEGRAM_ENABLE
  } else if (mode == 3) {
    sensor->resetExtremumsEntirely();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_ENTIRELY_DEV, sensor_name);
    #endif // CONFIG_TELEGRAM_ENABLE
  };
}

static void sensorsResetExtremumsSensors(uint8_t mode)
{
  if (mode == 0) {
    sensorOutdoor.resetExtremumsTotal();
    sensorIndoor.resetExtremumsTotal();
    sensorBoiler.resetExtremumsTotal();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_TOTAL_ALL);
    #endif // CONFIG_TELEGRAM_ENABLE
  } else if (mode == 1) {
    sensorOutdoor.resetExtremumsDaily();
    sensorIndoor.resetExtremumsDaily();
    sensorBoiler.resetExtremumsDaily();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_DAILY_ALL);
    #endif // CONFIG_TELEGRAM_ENABLE
  } else if (mode == 2) {
    sensorOutdoor.resetExtremumsWeekly();
    sensorIndoor.resetExtremumsWeekly();
    sensorBoiler.resetExtremumsWeekly();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_WEEKLY_ALL);
    #endif // CONFIG_TELEGRAM_ENABLE
  } else if (mode == 3) {
    sensorOutdoor.resetExtremumsEntirely();
    sensorIndoor.resetExtremumsEntirely();
    sensorBoiler.resetExtremumsEntirely();
    #if CONFIG_TELEGRAM_ENABLE
      tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
        CONFIG_MESSAGE_TG_SENSOR_CLREXTR_ENTIRELY_ALL);
    #endif // CONFIG_TELEGRAM_ENABLE
  };
};

static void sensorsCommandsEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if ((event_id == RE_SYS_COMMAND) && (event_data)) {
    char* buf = malloc_string((char*)event_data);
    if (buf != nullptr) {
      const char* seps = " ";
      char* cmd = nullptr;
      char* mode = nullptr;
      char* sensor = nullptr;
      uint8_t imode = 0;
      cmd = strtok(buf, seps);
      if ((cmd != nullptr) && (strcasecmp(cmd, CONFIG_SENSOR_COMMAND_EXTR_RESET) == 0)) {
        rlog_i(logTAG, "Reset extremums: %s", buf);
        sensor = strtok(nullptr, seps);
        if (sensor != nullptr) {
          mode = strtok(nullptr, seps);
        };
      
        // Опрделение режима сброса
        if (mode == nullptr) {
          // Возможно, вторым токеном идет режим, в этом случае сбрасываем для всех сенсоров
          if (sensor) {
            if (strcasecmp(sensor, CONFIG_SENSOR_EXTREMUMS_DAILY) == 0) {
              sensor = nullptr;
              imode = 1;
            } else if (strcasecmp(sensor, CONFIG_SENSOR_EXTREMUMS_WEEKLY) == 0) {
              sensor = nullptr;
              imode = 2;
            } else if (strcasecmp(sensor, CONFIG_SENSOR_EXTREMUMS_ENTIRELY) == 0) {
              sensor = nullptr;
              imode = 3;
            };
          };
        } else if (strcasecmp(mode, CONFIG_SENSOR_EXTREMUMS_DAILY) == 0) {
          imode = 1;
        } else if (strcasecmp(mode, CONFIG_SENSOR_EXTREMUMS_WEEKLY) == 0) {
          imode = 2;
        } else if (strcasecmp(mode, CONFIG_SENSOR_EXTREMUMS_ENTIRELY) == 0) {
          imode = 3;
        };

        // Определение сенсора
        if ((sensor == nullptr) || (strcasecmp(sensor, CONFIG_SENSOR_COMMAND_SENSORS_PREFIX) == 0)) {
          sensorsResetExtremumsSensors(imode);
        } else {
          if (strcasecmp(sensor, SENSOR_OUTDOOR_TOPIC) == 0) {
            sensorsResetExtremumsSensor(&sensorOutdoor, SENSOR_OUTDOOR_TOPIC, imode);
          } else if (strcasecmp(sensor, SENSOR_INDOOR_TOPIC) == 0) {
            sensorsResetExtremumsSensor(&sensorIndoor, SENSOR_INDOOR_TOPIC, imode);
          } else if (strcasecmp(sensor, SENSOR_BOILER_TOPIC) == 0) {
            sensorsResetExtremumsSensor(&sensorBoiler, SENSOR_BOILER_TOPIC, imode);
          } else {
            rlog_w(logTAG, "Sensor [ %s ] not found", sensor);
            #if CONFIG_TELEGRAM_ENABLE
              tgSend(CONFIG_SENSOR_COMMAND_KIND, CONFIG_SENSOR_COMMAND_PRIORITY, CONFIG_SENSOR_COMMAND_NOTIFY, CONFIG_TELEGRAM_DEVICE,
                CONFIG_MESSAGE_TG_SENSOR_CLREXTR_UNKNOWN, sensor);
            #endif // CONFIG_TELEGRAM_ENABLE
          };
        };
      };
    };
    if (buf != nullptr) free(buf);
  };
}

static void sensorsOtaEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if ((event_id == RE_SYS_OTA) && (event_data)) {
    re_system_event_data_t* data = (re_system_event_data_t*)event_data;
    if (data->type == RE_SYS_SET) {
      sensorsTaskSuspend();
    } else {
      sensorsTaskResume();
    };
  };
}

bool sensorsEventHandlersRegister()
{
  return eventHandlerRegister(RE_MQTT_EVENTS, ESP_EVENT_ANY_ID, &sensorsMqttEventHandler, nullptr) 
      && eventHandlerRegister(RE_TIME_EVENTS, RE_TIME_START_OF_DAY, &sensorsTimeEventHandler, nullptr)
      && eventHandlerRegister(RE_SYSTEM_EVENTS, RE_SYS_COMMAND, &sensorsCommandsEventHandler, nullptr)
      && eventHandlerRegister(RE_SYSTEM_EVENTS, RE_SYS_OTA, &sensorsOtaEventHandler, nullptr);
}


// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- Дисплей -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// Инициализация LCD перенесена в lcdDisplayInit() внутри sensorsTaskExec,
// чтобы избежать двойного вызова lcd.init() и гонки по I2C шине.
// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- Задача --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void sensorsTaskExec(void *pvParameters)
{
  static TickType_t prevTicks = xTaskGetTickCount();

  // -------------------------------------------------------------------------------------------------------
  // Инициализация параметров
  // -------------------------------------------------------------------------------------------------------
  sensorsInitParameters();

  // -------------------------------------------------------------------------------------------------------
  // Инициализация сенсоров 
  // -------------------------------------------------------------------------------------------------------
  sensorsInitSensors();

  // -------------------------------------------------------------------------------------------------------
  // Инициализация термостата 
  // -------------------------------------------------------------------------------------------------------
  sensorsInitRelays();

  // -------------------------------------------------------------------------------------------------------
  // Инициализация контроллеров
  // -------------------------------------------------------------------------------------------------------
  // Инициализация контроллеров OpenMon
  #if CONFIG_OPENMON_ENABLE
    dsChannelInit(EDS_OPENMON, 
      CONFIG_OPENMON_CTR01_ID, CONFIG_OPENMON_CTR01_TOKEN, 
      CONFIG_OPENMON_MIN_INTERVAL, CONFIG_OPENMON_ERROR_INTERVAL);
  #endif // CONFIG_OPENMON_ENABLE
  
  // Инициализация контроллеров NarodMon
  #if CONFIG_NARODMON_ENABLE
    dsChannelInit(EDS_NARODMON, 
      CONFIG_NARODMON_DEVICE01_ID, CONFIG_NARODMON_DEVICE01_KEY, 
      CONFIG_NARODMON_MIN_INTERVAL, CONFIG_NARODMON_ERROR_INTERVAL);
  #endif // CONFIG_NARODMON_ENABLE

  // Инициализация каналов ThingSpeak
  #if CONFIG_THINGSPEAK_ENABLE
    dsChannelInit(EDS_THINGSPEAK, 
      CONFIG_THINGSPEAK_CHANNEL01_ID, CONFIG_THINGSPEAK_CHANNEL01_KEY, 
      CONFIG_THINGSPEAK_MIN_INTERVAL, CONFIG_THINGSPEAK_ERROR_INTERVAL);
  #endif // CONFIG_THINGSPEAK_ENABLE

  // -------------------------------------------------------------------------------------------------------
  // Таймеры публикции данных с сенсоров
  // -------------------------------------------------------------------------------------------------------
  esp_timer_t mqttPubTimer;
  timerSet(&mqttPubTimer, iMqttPubInterval*1000);
  #if CONFIG_OPENMON_ENABLE
    esp_timer_t omSendTimer;
    timerSet(&omSendTimer, iOpenMonInterval*1000);
  #endif // CONFIG_OPENMON_ENABLE
  #if CONFIG_NARODMON_ENABLE
    esp_timer_t nmSendTimer;
    timerSet(&nmSendTimer, iNarodMonInterval*1000);
  #endif // CONFIG_NARODMON_ENABLE
  #if CONFIG_THINGSPEAK_ENABLE
    esp_timer_t tsSendTimer;
    timerSet(&tsSendTimer, iThingSpeakInterval*1000);
  #endif // CONFIG_THINGSPEAK_ENABLE

  // Initialize LCD display
  if (!lcdDisplayInit()) {
    rlog_e(logTAG, "Failed to initialize LCD display");
  } else if (!lcdDisplayStart()) {
    rlog_e(logTAG, "Failed to start LCD display task");
  }

  // Счётчик последовательных сбоев датчика улицы.
  // DHT22 при температурах около 0°C выдаёт два типа мусора:
  //   1. Знаковый артефакт: -3272°C (raw вне диапазона -50..60)
  //   2. Пустой фрейм: 0.00°C / 0.00% (все биты нуль — BitError или потеря синхронизации)
  // При накоплении SENSOR_OUTDOOR_FILTER_SIZE подряд идущих сбоев медианный буфер полностью
  // "отравляется" мусором. Принудительный сброс через doChangeFilterMode() заставляет
  // следующее нормальное значение сразу заполнить весь буфер — медиана мгновенно восстанавливается.
  static const uint8_t OUTDOOR_OUTLIER_RESET_THRESHOLD = SENSOR_OUTDOOR_FILTER_SIZE;
  uint8_t outdoorOutlierCount = 0;

  while (1) {
    // -----------------------------------------------------------------------------------------------------
    // Чтение данных с сенсоров
    // -----------------------------------------------------------------------------------------------------
    sensorOutdoor.readData();
    if (sensorOutdoor.getStatus() == SENSOR_STATUS_OK) {
      float outdoorRaw  = sensorOutdoor.getValue(false).rawValue; // температура (до offset)
      // DS18B20 не имеет пустых фреймов типа DHT22, но нулевая температура теоретически возможна реальная.
      // Оставляем только проверку диапазона.
      bool isZeroFrame  = false; // DS18B20: нет протокольных пустых фреймов
      bool isOutOfRange = (outdoorRaw < SENSOR_OUTDOOR_TEMP_MIN) || (outdoorRaw > SENSOR_OUTDOOR_TEMP_MAX);
      if (isOutOfRange || isZeroFrame) {
        outdoorOutlierCount++;
        _outdoorDataValid = false;
        if (isZeroFrame) {
          rlog_w("OUTDOOR", "Zero frame (empty DHT22 response), streak: %d/%d",
            outdoorOutlierCount, OUTDOOR_OUTLIER_RESET_THRESHOLD);
        } else {
          rlog_w("OUTDOOR", "Out of range value rejected: %.2f C (valid: %.1f..%.1f), streak: %d/%d",
            outdoorRaw, SENSOR_OUTDOOR_TEMP_MIN, SENSOR_OUTDOOR_TEMP_MAX,
            outdoorOutlierCount, OUTDOOR_OUTLIER_RESET_THRESHOLD);
        }
        // Если медианный буфер полностью забит мусором — сбрасываем его.
        // После сброса (_filterInit=false) первое нормальное значение заполнит весь буфер,
        // и медиана сразу вернёт корректный результат без "выплёскивания" мусора.
        if (outdoorOutlierCount >= OUTDOOR_OUTLIER_RESET_THRESHOLD) {
          rSensorItem* tempItem = sensorOutdoor.getSensorItem();
          if (tempItem != nullptr) {
            tempItem->doChangeFilterMode();
            rlog_w("OUTDOOR", "Median filter reset after %d consecutive bad values", outdoorOutlierCount);
          }
          outdoorOutlierCount = 0;
        }
      } else {
        outdoorOutlierCount = 0;
        _outdoorDataValid = true;
        rlog_i("OUTDOOR", "Values raw: %.2f C | out: %.2f C | min: %.2f C | max: %.2f C",
          outdoorRaw,
          sensorOutdoor.getValue(false).filteredValue,
          sensorOutdoor.getExtremumsDaily(false).minValue.filteredValue,
          sensorOutdoor.getExtremumsDaily(false).maxValue.filteredValue);
      };
    };
    sensorIndoor.readData();
    if (sensorIndoor.getStatus() == SENSOR_STATUS_OK) {
      rlog_i("INDOOR", "Values raw: %.2f °С / %.2f mmhg | out: %.2f °С / %.2f mmhg | min: %.2f °С / %.2f mmhg | max: %.2f °С / %.2f mmhg", 
        sensorIndoor.getValue2(false).rawValue, sensorIndoor.getValue1(false).rawValue, 
        sensorIndoor.getValue2(false).filteredValue, sensorIndoor.getValue1(false).filteredValue, 
        sensorIndoor.getExtremumsDaily2(false).minValue.filteredValue, sensorIndoor.getExtremumsDaily1(false).minValue.filteredValue, 
        sensorIndoor.getExtremumsDaily2(false).maxValue.filteredValue, sensorIndoor.getExtremumsDaily1(false).maxValue.filteredValue);
    };
    sensorBoiler.readData();
    if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
      rlog_i("BOILER", "Values raw: %.2f °С | out: %.2f °С | min: %.2f °С | max: %.2f °С", 
        sensorBoiler.getValue(false).rawValue, 
        sensorBoiler.getValue(false).filteredValue,
        sensorBoiler.getExtremumsDaily(false).minValue.filteredValue,
        sensorBoiler.getExtremumsDaily(false).maxValue.filteredValue);
    };

    // -----------------------------------------------------------------------------------------------------
    // Контроль температуры
    // -----------------------------------------------------------------------------------------------------

    sensorsBoilerControl();

    if (sensorIndoor.getStatus() == SENSOR_STATUS_OK) {
      tempMonitorIndoor.checkValue(sensorIndoor.getValue2(false).filteredValue);
    };
    if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
      tempMonitorBoiler.checkValue(sensorBoiler.getValue(false).filteredValue);
    };

    // -----------------------------------------------------------------------------------------------------
    // Передача данных на LCD дисплей (без прямого доступа к сенсорам из LCD-задачи)
    // -----------------------------------------------------------------------------------------------------
    {
      float ind_t = (sensorIndoor.getStatus() == SENSOR_STATUS_OK)  ? sensorIndoor.getValue2(false).filteredValue  : NAN;
      float out_t_raw = (sensorOutdoor.getStatus() == SENSOR_STATUS_OK) ? sensorOutdoor.getValue(false).filteredValue : NAN;
      // outdoorDataValid=false если последнее чтение было мусором (out-of-range или zero frame).
      // Это предотвращает отображение "0.0" на LCD при пустом фрейме DHT22.
      float out_t = (_outdoorDataValid && !isnan(out_t_raw) && (out_t_raw >= SENSOR_OUTDOOR_TEMP_MIN) && (out_t_raw <= SENSOR_OUTDOOR_TEMP_MAX)) ? out_t_raw : NAN;
      float boi_t = (sensorBoiler.getStatus() == SENSOR_STATUS_OK)  ? sensorBoiler.getValue(false).filteredValue   : NAN;

      char modeChar = '0';
      float targetTemp = thermostatInternalTemp;
      if      (thermostatMode == THERMOSTAT_OFF)           { modeChar = '0'; }
      else if (thermostatMode == THERMOSTAT_ON)            { modeChar = '1'; }
      else if (thermostatMode == THERMOSTAT_TIME)          { modeChar = '2'; }
      else if (thermostatMode == THERMOSTAT_TEMP)          { modeChar = '3'; }
      else if (thermostatMode == THERMOSTAT_TIME_AND_TEMP) { modeChar = '4'; }
      else if (thermostatMode == THERMOSTAT_INTERPOLATION) {
        modeChar = '5';
        if (!isnan(out_t) && interpolation_points_count >= 2) {
          targetTemp = calculateInterpolatedTemp(out_t, interpolation_points, interpolation_points_count);
        }
      }

      lcdUpdateData(
        ind_t, !isnan(ind_t),
        out_t, !isnan(out_t),
        boi_t, !isnan(boi_t),
        lcBoiler.getState(),
        targetTemp,
        modeChar
      );
    };

    // -----------------------------------------------------------------------------------------------------
    // Сохранение данных сенсоров
    // -----------------------------------------------------------------------------------------------------

    if (_sensorsNeedStore) {
      _sensorsNeedStore = false;
      sensorsStoreData();
    };

    // -----------------------------------------------------------------------------------------------------
    // Публикация данных с сенсоров
    // -----------------------------------------------------------------------------------------------------

    // MQTT брокер
    if (esp_heap_free_check() && statesMqttIsConnected() && timerTimeout(&mqttPubTimer)) {
      timerSet(&mqttPubTimer, iMqttPubInterval*1000);
      sensorOutdoor.publishData(false);
      sensorIndoor.publishData(false);
      tempMonitorIndoor.mqttPublish();
      sensorBoiler.publishData(false);
      tempMonitorBoiler.mqttPublish();
      lcBoiler.mqttPublish();
    };

    // open-monitoring.online
    #if CONFIG_OPENMON_ENABLE
      if (statesInetIsAvailabled() && timerTimeout(&omSendTimer)) {
        timerSet(&omSendTimer, iOpenMonInterval*1000);
        char * omValues = nullptr;
        // Улица — DS18B20, только температура
        if (sensorOutdoor.getStatus() == SENSOR_STATUS_OK) {
          omValues = concat_strings_div(omValues, 
            malloc_stringf("p1=%.3f", 
              sensorOutdoor.getValue(false).filteredValue),
            "&");
        };
        // Комната
        if (sensorIndoor.getStatus() == SENSOR_STATUS_OK) {
          omValues = concat_strings_div(omValues, 
            malloc_stringf("p3=%.3f&p4=%.2f", 
              sensorIndoor.getValue2(false).filteredValue, sensorIndoor.getValue1(false).filteredValue),
            "&");
        };
        // Котёл
        if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
          omValues = concat_strings_div(omValues, 
            malloc_stringf("p5=%.3f", 
              sensorBoiler.getValue(false).filteredValue),
            "&");
        };
        // Отправляем данные
        if (omValues) {
          dsSend(EDS_OPENMON, CONFIG_OPENMON_CTR01_ID, omValues, false); 
          free(omValues);
        };
      };
    #endif // CONFIG_OPENMON_ENABLE

    // narodmon.ru
    #if CONFIG_NARODMON_ENABLE
      if (statesInetIsAvailabled() && timerTimeout(&nmSendTimer)) {
        timerSet(&nmSendTimer, iNarodMonInterval*1000);
        char * nmValues = nullptr;
        // Улица — DS18B20, только температура
        if (sensorOutdoor.getStatus() == SENSOR_STATUS_OK) {
          nmValues = concat_strings_div(nmValues, 
            malloc_stringf("Tout=%.2f", 
              sensorOutdoor.getValue(false).filteredValue),
            "&");
        };
        // Комната
        if (sensorIndoor.getStatus() == SENSOR_STATUS_OK) {
          nmValues = concat_strings_div(nmValues, 
            malloc_stringf("Tin=%.2f&Hin=%.2f", 
              sensorIndoor.getValue2(false).filteredValue, sensorIndoor.getValue1(false).filteredValue),
            "&");
        };
        // Котёл
        if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
          nmValues = concat_strings_div(nmValues, 
            malloc_stringf("Tboiler=%.2f", 
              sensorBoiler.getValue(false).filteredValue),
            "&");
        };
        // Отправляем данные
        if (nmValues) {
          dsSend(EDS_NARODMON, CONFIG_NARODMON_DEVICE01_ID, nmValues, false); 
          free(nmValues);
        };
      };
    #endif // CONFIG_NARODMON_ENABLE

    // thingspeak.com
    #if CONFIG_THINGSPEAK_ENABLE
      if (statesInetIsAvailabled() && timerTimeout(&tsSendTimer)) {
        timerSet(&tsSendTimer, iThingSpeakInterval*1000);

        char * tsValues = nullptr;
        // Улица
        if (sensorOutdoor.getStatus() == SENSOR_STATUS_OK) {
          tsValues = concat_strings_div(tsValues, 
            malloc_stringf("field1=%.3f", 
              sensorOutdoor.getValue(false).filteredValue),
            "&");
        };
        // Комната
        if (sensorIndoor.getStatus() == SENSOR_STATUS_OK) {
          tsValues = concat_strings_div(tsValues, 
            malloc_stringf("field3=%.3f&field4=%.2f", 
              sensorIndoor.getValue2(false).filteredValue, sensorIndoor.getValue1(false).filteredValue),
            "&");
        };
        // Котёл
        if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
          tsValues = concat_strings_div(tsValues, 
            malloc_stringf("field5=%.3f", 
              sensorBoiler.getValue(false).filteredValue),
            "&");
        };
        // Отправляем данные
        if (tsValues) {
          dsSend(EDS_THINGSPEAK, CONFIG_THINGSPEAK_CHANNEL01_ID, tsValues, false); 
          free(tsValues);
        };
      };
    #endif // CONFIG_THINGSPEAK_ENABLE
    
    // -----------------------------------------------------------------------------------------------------
    // Ожидание
    // -----------------------------------------------------------------------------------------------------
    vTaskDelayUntil(&prevTicks, pdMS_TO_TICKS(CONFIG_SENSORS_TASK_CYCLE));
  };

  vTaskDelete(nullptr);
  espRestart(RR_UNKNOWN);
}

bool sensorsTaskStart()
{
  #if CONFIG_SENSORS_STATIC_ALLOCATION
    static StaticTask_t sensorsTaskBuffer;
    static StackType_t sensorsTaskStack[CONFIG_SENSORS_TASK_STACK_SIZE];
    _sensorsTask = xTaskCreateStaticPinnedToCore(sensorsTaskExec, sensorsTaskName, 
      CONFIG_SENSORS_TASK_STACK_SIZE, NULL, CONFIG_TASK_PRIORITY_SENSORS, sensorsTaskStack, &sensorsTaskBuffer, CONFIG_TASK_CORE_SENSORS);
  #else
    xTaskCreatePinnedToCore(sensorsTaskExec, sensorsTaskName, 
      CONFIG_SENSORS_TASK_STACK_SIZE, NULL, CONFIG_TASK_PRIORITY_SENSORS, &_sensorsTask, CONFIG_TASK_CORE_SENSORS);
  #endif // CONFIG_SENSORS_STATIC_ALLOCATION
  if (_sensorsTask) {
    rloga_i("Task [ %s ] has been successfully created and started", sensorsTaskName);
    return sensorsEventHandlersRegister();
  }
  else {
    rloga_e("Failed to create a task for processing sensor readings!");
    return false;
  };
}

bool sensorsTaskSuspend()
{
  if ((_sensorsTask) && (eTaskGetState(_sensorsTask) != eSuspended)) {
    vTaskSuspend(_sensorsTask);
    if (eTaskGetState(_sensorsTask) == eSuspended) {
      rloga_d("Task [ %s ] has been suspended", sensorsTaskName);
      return true;
    } else {
      rloga_e("Failed to suspend task [ %s ]!", sensorsTaskName);
    };
  };
  return false;
}

bool sensorsTaskResume()
{
  if ((_sensorsTask) && (eTaskGetState(_sensorsTask) == eSuspended)) {
    vTaskResume(_sensorsTask);
    if (eTaskGetState(_sensorsTask) != eSuspended) {
      rloga_i("Task [ %s ] has been successfully resumed", sensorsTaskName);
      return true;
    } else {
      rloga_e("Failed to resume task [ %s ]!", sensorsTaskName);
    };
  };
  return false;
}

float calculateInterpolatedTemp(float outdoor_temp, interpolation_point_t* points, uint8_t points_count)
{
  if ((points == nullptr) || (points_count < 2)) {
    return 0.0f;
  }

  // Находим две ближайшие точки для интерполяции
  int i = 0;
  while ((i < (int)points_count - 1) && (outdoor_temp > points[i + 1].outdoor_temp)) {
    i++;
  }

  // Если температура ниже минимальной или выше максимальной, используем крайние значения
  if ((i == 0) && (outdoor_temp <= points[0].outdoor_temp)) {
    return points[0].boiler_temp;
  }
  if ((i == (int)points_count - 1) && (outdoor_temp >= points[i].outdoor_temp)) {
    return points[i].boiler_temp;
  }

  // Линейная интерполяция между двумя соседними точками
  const float t1  = points[i].outdoor_temp;
  const float t2  = points[i + 1].outdoor_temp;
  const float tp1 = points[i].boiler_temp;
  const float tp2 = points[i + 1].boiler_temp;

  if (t2 == t1) {
    return tp1;
  }

  return tp1 + (tp2 - tp1) * (outdoor_temp - t1) / (t2 - t1);
}
