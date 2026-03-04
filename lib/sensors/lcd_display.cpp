#include "lcd_display.h"
#include "rLog.h"
#include <math.h>     // NAN, isnan
#include <string.h>   // memcpy
// sensors.h НЕ включается здесь — данные передаются через lcdUpdateData(),
// чтобы LCD-задача не обращалась к объектам сенсоров напрямую из другого потока.

static const char* logTAG = "LCD";

// Task handle
static TaskHandle_t _lcdTask = nullptr;

// Данные для отображения — обновляются из задачи сенсоров через lcdUpdateData()
// Не volatile: защита обеспечивается мьютексом portMUX, volatile здесь не нужен
static lcd_display_data_t _displayData = {
    .indoor_temp = 0.0f,
    .outdoor_temp = 0.0f,
    .boiler_temp = 0.0f,
    .indoor_valid = false,
    .outdoor_valid = false,
    .boiler_valid = false,
    .boiler_state = false,
    .thermostat_temp = 0.0f,
    .mode_char = '@'
};

// Мьютекс для защиты _displayData от одновременного доступа двух задач
static portMUX_TYPE _displayDataMux = portMUX_INITIALIZER_UNLOCKED;

// Вызывается из sensorsTaskExec — обновляет данные для дисплея без I2C из LCD-задачи
void lcdUpdateData(
    float indoor_temp, bool indoor_valid,
    float outdoor_temp, bool outdoor_valid,
    float boiler_temp, bool boiler_valid,
    bool boiler_state,
    float thermostat_temp,
    char mode_char)
{
    taskENTER_CRITICAL(&_displayDataMux);
    _displayData.indoor_temp    = indoor_temp;
    _displayData.indoor_valid   = indoor_valid;
    _displayData.outdoor_temp   = outdoor_temp;
    _displayData.outdoor_valid  = outdoor_valid;
    _displayData.boiler_temp    = boiler_temp;
    _displayData.boiler_valid   = boiler_valid;
    _displayData.boiler_state   = boiler_state;
    _displayData.thermostat_temp = thermostat_temp;
    _displayData.mode_char      = mode_char;
    taskEXIT_CRITICAL(&_displayDataMux);
}

// Initialize LCD display — вызывается один раз из sensorsTaskExec
bool lcdDisplayInit() {
    lcd.begin(16, 2);
    lcd.init();
    lcd.clear();
    lcd.setBacklight(true);
    lcd.printpos(0, 0, "Loading...");
    lcd.printpos(0, 1, APP_VERSION);
    return true;
}

// Start LCD display task
bool lcdDisplayStart() {
    if (_lcdTask != nullptr) {
        return true;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        lcdDisplayTask,
        "lcd_display",
        CONFIG_LCD_TASK_STACK_SIZE,
        nullptr,
        CONFIG_TASK_PRIORITY_SENSORS - 1,  // приоритет чуть ниже задачи сенсоров
        &_lcdTask,
        CONFIG_TASK_CORE_SENSORS
    );

    if (result != pdPASS) {
        rlog_e(logTAG, "Failed to create LCD display task");
        return false;
    }

    return true;
}

// Stop LCD display task
bool lcdDisplayStop() {
    if (_lcdTask != nullptr) {
        vTaskDelete(_lcdTask);
        _lcdTask = nullptr;
    }
    return true;
}

// LCD display task — только читает _displayData и рисует на экране, датчики не трогает
void lcdDisplayTask(void* arg) {
    char line1[17];
    char line2[17];
    lcd_display_data_t localData;

    while (1) {
        // Копируем данные под защитой мьютекса
        taskENTER_CRITICAL(&_displayDataMux);
        memcpy(&localData, &_displayData, sizeof(lcd_display_data_t));
        taskEXIT_CRITICAL(&_displayDataMux);

        // Форматируем и выводим на дисплей
        lcdDisplayFormatData(localData, line1, line2);

        // LCD обновляется реже чем опрос сенсоров — каждые 5 секунд
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void lcdDisplayFormatData(const lcd_display_data_t& data, char* line1, char* line2) {
    char line1_content[17];
    char line2_content[17];

    // Строка 1: R=XX.X O=XX.X
    if (data.indoor_valid && data.outdoor_valid) {
        snprintf(line1_content, 17, "R=%.1f O=%.1f", data.indoor_temp, data.outdoor_temp);
    } else if (data.indoor_valid) {
        snprintf(line1_content, 17, "R=%.1f O=Na", data.indoor_temp);
    } else if (data.outdoor_valid) {
        snprintf(line1_content, 17, "R=Na O=%.1f", data.outdoor_temp);
    } else {
        snprintf(line1_content, 17, "R=Na O=Na");
    }
    snprintf(line1, 17, "%-16s", line1_content);

    // Строка 2: B=XX.X [ON/OF][mode] XX.X
    const char* state = data.boiler_state ? "ON" : "OF";
    if (data.boiler_valid) {
        snprintf(line2_content, 17, "B=%.1f %s%c %.1f",
            data.boiler_temp,
            state,
            data.mode_char,
            data.thermostat_temp);
    } else {
        snprintf(line2_content, 17, "B=Na %s%c %.1f",
            state,
            data.mode_char,
            data.thermostat_temp);
    }
    snprintf(line2, 17, "%-16s", line2_content);

    // Обновляем дисплей без lcd.clear() чтобы избежать мерцания
    // Пробелы в конце строк затирают остатки предыдущего текста
    lcd.printpos(0, 0, line1);
    lcd.printpos(0, 1, line2);
} 