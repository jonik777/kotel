/* 
   EN: Class for monitoring the state of a digital input or any other binary object (door, gate, faucet, switch, etc.) with JSON publishing on an MQTT broker
   RU: Класс для контроля состояния цифрового входа или любого друого двоичного объекта (дверь, ворота, кран, выключатель и т.д.) с публикацией JSON на MQTT-брокере
   --------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Страница проекта: https://github.com/kotyara12/reDInStat
*/

#ifndef __RE_BIN_STAT_H__
#define __RE_BIN_STAT_H__

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

class rBinStat;

typedef void (*cb_bin_change_t) (rBinStat *bin, bool state, time_t duration);
typedef bool (*cb_bin_publish_t) (rBinStat *bin, char* topic, char* payload, bool free_topic, bool free_payload);

class rBinStat {
  public:
    rBinStat(bool init_state, cb_bin_change_t cb_state_changed, cb_bin_publish_t cb_mqtt_publish);
    ~rBinStat();

    // State change
    bool Change(bool new_state, bool forced, bool publish);

    // Current state
    uint8_t getState();
    time_t getLastChange();
    time_t getLastTrue();
    time_t getLastFalse();

    // Generate JSON
    char* getTimestampsJSON();
    char* getJSON();

    // MQTT
    void mqttSetCallback(cb_bin_publish_t cb_publish);
    char* mqttTopicGet();
    bool mqttTopicSet(char* topic);
    bool mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3);
    void mqttTopicFree();
    bool mqttPublish();
  private:
    bool                  _state = false;           // Current state
    time_t                _last_true = 0;           // Time of last true
    time_t                _last_false = 0;          // Time of last false
    char*                 _mqtt_topic = nullptr;    // MQTT topic

    cb_bin_change_t       _on_changed = nullptr;   // Pointer to the callback function to be called on change state
    cb_bin_publish_t      _mqtt_publish = nullptr; // Pointer to the publish callback function
};

#ifdef __cplusplus
}
#endif

#endif // __RE_BIN_STAT_H__