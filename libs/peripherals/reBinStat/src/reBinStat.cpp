#include "reBinStat.h"
#include <string.h>
#include "reEsp32.h"
#include "rLog.h"
#include "rStrings.h"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ rBinStat -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rBinStat::rBinStat(bool init_state, cb_bin_change_t cb_state_changed, cb_bin_publish_t cb_mqtt_publish)
{
  _state = init_state;
  _on_changed = cb_state_changed;
  _mqtt_publish = cb_mqtt_publish;
  _mqtt_topic = nullptr;
}

rBinStat::~rBinStat()
{
}

// State change
bool rBinStat::Change(bool new_state, bool forced, bool publish)
{
  if (forced || (new_state != _state)) {
    _state = new_state;
    if (_state) {
      _last_true = time(nullptr);
    } else {
      _last_false = time(nullptr);
    };
    
    // Call нandlers
    if (_on_changed) {
      time_t duration = 0;
      if ((_last_true > 1000000000) && (_last_false > 1000000000)) {
        duration = _state ? _last_false - _last_true : _last_true - _last_false;
      };
      _on_changed(this, _state, duration);
    };
    if (publish) {
      mqttPublish();
    };
    return true;
  };
  return false;
}

// Current state
uint8_t rBinStat::getState()
{
  return _state;
}

time_t rBinStat::getLastChange()
{
  return _last_true > _last_false ? _last_true : _last_false;
}

time_t rBinStat::getLastTrue()
{
  return _last_true;
}

time_t rBinStat::getLastFalse()
{
  return _last_false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- MQTT ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rBinStat::mqttSetCallback(cb_bin_publish_t cb_publish)
{
  _mqtt_publish = cb_publish;
}

char* rBinStat::mqttTopicGet()
{
  return _mqtt_topic;
}

bool rBinStat::mqttTopicSet(char* topic)
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = topic;
  return (_mqtt_topic != nullptr);
}

bool rBinStat::mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3)
{
  return mqttTopicSet(mqttGetTopicDevice(primary, local, topic1, topic2, topic3));
}

void rBinStat::mqttTopicFree()
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

bool rBinStat::mqttPublish()
{
  if ((_mqtt_topic) && (_mqtt_publish)) {
    return _mqtt_publish(this, _mqtt_topic, getJSON(), false, true);
  };
  return false;
}

char* rBinStat::getTimestampsJSON()
{
  char _time_changed[CONFIG_BINSTAT_TIMESTAMP_BUF_SIZE];
  char _time_true[CONFIG_BINSTAT_TIMESTAMP_BUF_SIZE];
  char _time_false[CONFIG_BINSTAT_TIMESTAMP_BUF_SIZE];
  
  time_t _last_changed = _last_true > _last_false ? _last_true : _last_false;
  time2str_empty( CONFIG_BINSTAT_TIMESTAMP_FORMAT, &_last_changed, &_time_changed[0], sizeof(_time_changed));
  time2str_empty( CONFIG_BINSTAT_TIMESTAMP_FORMAT, &_last_true, &_time_true[0], sizeof(_time_true));
  time2str_empty( CONFIG_BINSTAT_TIMESTAMP_FORMAT, &_last_false, &_time_false[0], sizeof(_time_false));

  return malloc_stringf("{\"" CONFIG_BINSTAT_CHANGED "\":\"%s\",\"" CONFIG_BINSTAT_TRUE "\":\"%s\",\"" CONFIG_BINSTAT_FALSE "\":\"%s\"}", _time_changed, _time_true, _time_false);
}

char* rBinStat::getJSON()
{
  char* _json = nullptr;
  char* _json_time = getTimestampsJSON();
  if (_json_time) {
    _json = malloc_stringf("{\"" CONFIG_BINSTAT_STATUS "\":%d,\"" CONFIG_BINSTAT_TIMESTAMP "\":%s}", _state, _json_time);
  };
  if (_json_time) free(_json_time);
  return _json;
}

