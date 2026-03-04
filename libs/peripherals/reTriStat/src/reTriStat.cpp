#include "reTriStat.h"
#include <string.h>
#include "reEsp32.h"
#include "rLog.h"
#include "rStrings.h"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ rTriStat -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rTriStat::rTriStat(uint8_t init_state, cb_tri_change_t cb_state_changed, cb_tri_publish_t cb_mqtt_publish)
{
  _state = init_state;
  _on_changed = cb_state_changed;
  _mqtt_publish = cb_mqtt_publish;
  _mqtt_topic = nullptr;
}

rTriStat::~rTriStat()
{
}

// State change
bool rTriStat::Change(uint8_t new_state, bool forced, bool publish)
{
  if (forced || (new_state != _state)) {
    _state = new_state;
    if (_state == 0) {
      _last_0 = time(nullptr);
    } else if (_state == 1) {
      _last_1 = time(nullptr);
    } else if (_state == 2) {
      _last_2 = time(nullptr);
    };
    
    // Call нandlers
    if (_on_changed) {
      _on_changed(this, _state);
    };
    if (publish) {
      mqttPublish();
    };
    return true;
  };
  return false;
}

// Current state
uint8_t rTriStat::getState()
{
  return _state;
}

time_t rTriStat::getLastChange()
{
  if (_state == 0) {
    return _last_0;
  } else if (_state == 1) {
    return _last_1;
  } else if (_state == 2) {
    return _last_2;
  };
  return 0;
}

time_t rTriStat::getLast0()
{
  return _last_0;
}

time_t rTriStat::getLast1()
{
  return _last_1;
}

time_t rTriStat::getLast2()
{
  return _last_2;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- MQTT ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rTriStat::mqttSetCallback(cb_tri_publish_t cb_publish)
{
  _mqtt_publish = cb_publish;
}

char* rTriStat::mqttTopicGet()
{
  return _mqtt_topic;
}

bool rTriStat::mqttTopicSet(char* topic)
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = topic;
  return (_mqtt_topic != nullptr);
}

bool rTriStat::mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3)
{
  return mqttTopicSet(mqttGetTopicDevice(primary, local, topic1, topic2, topic3));
}

void rTriStat::mqttTopicFree()
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

bool rTriStat::mqttPublish()
{
  if ((_mqtt_topic) && (_mqtt_publish)) {
    return _mqtt_publish(this, _mqtt_topic, getJSON(), false, true);
  };
  return false;
}

char* rTriStat::getTimestampsJSON()
{
  char _time_changed[CONFIG_TRISTAT_TIMESTAMP_BUF_SIZE];
  char _time_0[CONFIG_TRISTAT_TIMESTAMP_BUF_SIZE];
  char _time_1[CONFIG_TRISTAT_TIMESTAMP_BUF_SIZE];
  char _time_2[CONFIG_TRISTAT_TIMESTAMP_BUF_SIZE];
  
  time_t _last_changed = getLastChange();
  time2str_empty( CONFIG_TRISTAT_TIMESTAMP_FORMAT, &_last_changed, &_time_changed[0], sizeof(_time_changed));
  time2str_empty( CONFIG_TRISTAT_TIMESTAMP_FORMAT, &_last_0, &_time_0[0], sizeof(_time_0));
  time2str_empty( CONFIG_TRISTAT_TIMESTAMP_FORMAT, &_last_1, &_time_1[0], sizeof(_time_1));
  time2str_empty( CONFIG_TRISTAT_TIMESTAMP_FORMAT, &_last_2, &_time_2[0], sizeof(_time_2));

  return malloc_stringf("{\"" CONFIG_TRISTAT_CHANGED "\":\"%s\",\"" CONFIG_TRISTAT_0 "\":\"%s\",\""  CONFIG_TRISTAT_1 "\":\"%s\",\"" CONFIG_TRISTAT_2 "\":\"%s\"}", 
    _time_changed, _time_0, _time_1, _time_2);
}

char* rTriStat::getJSON()
{
  char* _json = nullptr;
  char* _json_time = getTimestampsJSON();
  if (_json_time) {
    _json = malloc_stringf("{\"" CONFIG_TRISTAT_STATUS "\":%d,\"" CONFIG_TRISTAT_TIMESTAMP "\":%s}", _state, _json_time);
  };
  if (_json_time) free(_json_time);
  return _json;
}

