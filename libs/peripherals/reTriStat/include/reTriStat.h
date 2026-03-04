/* 
   EN: A class for monitoring the status of two digital inputs or any other tri-state object (water level, etc.) with JSON publishing for an MQTT broker
   RU: Класс для мониторинга состояния двух цифровых входов или любого другого объекта с тремя состояниями (уровень воды и т. д.) с публикацией JSON для брокера MQTT
   --------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Страница проекта: https://github.com/kotyara12/reDInStat
*/

#ifndef __RE_TRI_STAT_H__
#define __RE_TRI_STAT_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include "project_config.h"
#include "def_consts.h"

#ifdef __cplusplus
extern "C" {
#endif

class rTriStat;

typedef void (*cb_tri_change_t) (rTriStat *tri, uint8_t state);
typedef bool (*cb_tri_publish_t) (rTriStat *tri, char* topic, char* payload, bool free_topic, bool free_payload);

class rTriStat {
  public:
    rTriStat(uint8_t init_state, cb_tri_change_t cb_state_changed, cb_tri_publish_t cb_mqtt_publish);
    ~rTriStat();

    // State change
    bool Change(uint8_t new_state, bool forced, bool publish);

    // Current state
    uint8_t getState();
    time_t getLastChange();
    time_t getLast0();
    time_t getLast1();
    time_t getLast2();

    // Generate JSON
    char* getTimestampsJSON();
    char* getJSON();

    // MQTT
    void mqttSetCallback(cb_tri_publish_t cb_publish);
    char* mqttTopicGet();
    bool mqttTopicSet(char* topic);
    bool mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3);
    void mqttTopicFree();
    bool mqttPublish();
  private:
    uint8_t               _state = 0x00;            // Current state
    time_t                _last_0 = 0;              // Time of last state = 0
    time_t                _last_1 = 0;              // Time of last state = 1
    time_t                _last_2 = 0;              // Time of last state = 2
    char*                 _mqtt_topic = nullptr;    // MQTT topic

    cb_tri_change_t       _on_changed = nullptr;   // Pointer to the callback function to be called on change state
    cb_tri_publish_t      _mqtt_publish = nullptr; // Pointer to the publish callback function
};

#ifdef __cplusplus
}
#endif

#endif // __RE_TRI_STAT_H__