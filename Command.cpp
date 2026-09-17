#include "Command.h"
#include "ChargerController.h"
#include <ctype.h>
#include <string.h>

namespace {

const __FlashStringHelper* stateToken(ChargerState state) {
  switch (state) {
    case ChargerState::STARTUP:             return F("STARTUP");
    case ChargerState::CHARGING:            return F("CHARGING");
    case ChargerState::FULL_OFF:            return F("FULL_OFF");
    case ChargerState::BATTERY_TEMP_OFF:    return F("BATTERY_TEMP_OFF");
    case ChargerState::INTERNAL_TEMP_OFF:   return F("INTERNAL_TEMP_OFF");
    case ChargerState::SENSOR_FAULT:        return F("BATTERY_TEMP_SENSOR_FAULT");
    case ChargerState::INTERNAL_TEMP_FAULT: return F("NANO_TEMP_SENSOR_FAULT");
    case ChargerState::VOLTAGE_FAULT:       return F("VOLTAGE_FAULT");
    case ChargerState::OVERVOLTAGE_FAULT:   return F("OVERVOLTAGE_FAULT");
    case ChargerState::REMOTE_OFF:          return F("REMOTE_OFF");
    default:                                return F("UNKNOWN");
  }
}

}  // namespace

Command::Command(ChargerController& charger)
  : _charger(charger),
    _serial(PIN_ESP_RX, PIN_ESP_TX),
    _length(0),
    _discardUntilNewline(false),
    _tempSyncActive(false),
    _tempSyncStartMs(0),
    _tempSyncLastSampleMs(0),
    _tempSyncSamples(0),
    _tempSyncDeltaSum(0.0f),
    _tempSyncCurrentDelta(NAN),
    _tempSyncAverageDelta(NAN) {
  _buffer[0] = '\0';
}

void Command::begin() {
  if (!ENABLE_ESP_COMMANDS) {
    return;
  }

  _serial.begin(ESP_COMMAND_BAUD);
  _serial.println(F("READY NANO_LEAD_ACID_CHARGER"));
}

void Command::update() {
  if (!ENABLE_ESP_COMMANDS) {
    return;
  }

  while (_serial.available() > 0) {
    handleChar(static_cast<char>(_serial.read()));
  }

  updateTempSync(millis());
}

void Command::handleChar(char c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    if (_discardUntilNewline) {
      _discardUntilNewline = false;
      _length = 0;
      _buffer[0] = '\0';
      return;
    }

    _buffer[_length] = '\0';
    processLine();
    _length = 0;
    _buffer[0] = '\0';
    return;
  }

  if (_discardUntilNewline) {
    return;
  }

  if (c == '\b' || c == 127) {
    if (_length > 0) {
      --_length;
      _buffer[_length] = '\0';
    }
    return;
  }

  if (!isprint(static_cast<unsigned char>(c))) {
    return;
  }

  if (_length >= (COMMAND_BUFFER_SIZE - 1U)) {
    sendError(F("LINE_TOO_LONG"));
    _discardUntilNewline = true;
    _length = 0;
    _buffer[0] = '\0';
    return;
  }

  _buffer[_length++] = c;
  _buffer[_length] = '\0';
}

void Command::normalizeLine() {
  char* start = _buffer;
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) {
    ++start;
  }

  if (start != _buffer) {
    memmove(_buffer, start, strlen(start) + 1U);
  }

  size_t len = strlen(_buffer);
  while (len > 0U && isspace(static_cast<unsigned char>(_buffer[len - 1U]))) {
    _buffer[--len] = '\0';
  }

  char normalized[COMMAND_BUFFER_SIZE];
  size_t out = 0;
  bool previousWasSpace = false;

  for (size_t i = 0; _buffer[i] != '\0' && out < (COMMAND_BUFFER_SIZE - 1U); ++i) {
    const unsigned char raw = static_cast<unsigned char>(_buffer[i]);

    if (isspace(raw)) {
      if (!previousWasSpace && out > 0U) {
        normalized[out++] = ' ';
      }
      previousWasSpace = true;
      continue;
    }

    normalized[out++] = static_cast<char>(toupper(raw));
    previousWasSpace = false;
  }

  if (out > 0U && normalized[out - 1U] == ' ') {
    --out;
  }

  normalized[out] = '\0';
  strncpy(_buffer, normalized, COMMAND_BUFFER_SIZE);
  _buffer[COMMAND_BUFFER_SIZE - 1U] = '\0';
}

void Command::processLine() {
  normalizeLine();

  if (_buffer[0] == '\0') {
    return;
  }

  if (strcmp(_buffer, "PING") == 0) {
    _serial.println(F("PONG"));
    return;
  }

  if (strcmp(_buffer, "STATUS") == 0 || strcmp(_buffer, "GET STATUS") == 0) {
    sendStatus();
    return;
  }

  if (strcmp(_buffer, "BATTERY") == 0 || strcmp(_buffer, "GET BATTERY") == 0) {
    sendBattery();
    return;
  }

  if (strcmp(_buffer, "TEMP") == 0 ||
      strcmp(_buffer, "TEMPERATURE") == 0 ||
      strcmp(_buffer, "GET TEMP") == 0) {
    sendTemperature();
    return;
  }

  if (strcmp(_buffer, "STATE") == 0 || strcmp(_buffer, "GET STATE") == 0) {
    sendState();
    return;
  }

  if (strcmp(_buffer, "TSYNC START") == 0 || strcmp(_buffer, "TEMP SYNC START") == 0) {
    startTempSync();
    return;
  }

  if (strcmp(_buffer, "TSYNC STATUS") == 0 || strcmp(_buffer, "TEMP SYNC STATUS") == 0) {
    sendTempSyncStatus();
    return;
  }

  if (strcmp(_buffer, "TSYNC STOP") == 0 || strcmp(_buffer, "TEMP SYNC STOP") == 0) {
    stopTempSync();
    return;
  }

  if (strcmp(_buffer, "STOP") == 0 ||
      strcmp(_buffer, "OFF") == 0 ||
      strcmp(_buffer, "CHARGE OFF") == 0) {
    _charger.setRemoteInhibit(true);
    _serial.println(F("OK MODE=STOP CHARGER=OFF"));
    return;
  }

  if (strcmp(_buffer, "AUTO") == 0 || strcmp(_buffer, "CHARGE AUTO") == 0) {
    _charger.setRemoteInhibit(false);
    _serial.println(F("OK MODE=AUTO"));
    return;
  }

  if (strcmp(_buffer, "HELP") == 0 || strcmp(_buffer, "?") == 0) {
    sendHelp();
    return;
  }

  sendError(F("UNKNOWN_COMMAND"));
}

void Command::updateTempSync(unsigned long nowMs) {
  if (!_tempSyncActive) {
    return;
  }

  if ((nowMs - _tempSyncLastSampleMs) < TEMP_SYNC_SAMPLE_INTERVAL_MS) {
    return;
  }

  _tempSyncLastSampleMs = nowMs;

  // Only valid sensor pairs are used. A bad reading is skipped rather than
  // corrupting the average.
  if (!_charger.temperatureValid() || !_charger.internalTemperatureValid()) {
    return;
  }

  const float externalC = _charger.batteryTemperatureC();
  const float nanoC = _charger.internalTemperatureC();
  const float deltaC = externalC - nanoC;

  _tempSyncCurrentDelta = deltaC;
  _tempSyncDeltaSum += deltaC;
  ++_tempSyncSamples;
  _tempSyncAverageDelta = _tempSyncDeltaSum / static_cast<float>(_tempSyncSamples);
}

void Command::startTempSync() {
  if (!_charger.temperatureValid() || !_charger.internalTemperatureValid()) {
    sendError(F("TSYNC_SENSOR_INVALID"));
    return;
  }

  _tempSyncActive = true;
  _tempSyncStartMs = millis();
  _tempSyncLastSampleMs = _tempSyncStartMs;
  _tempSyncSamples = 0;
  _tempSyncDeltaSum = 0.0f;
  _tempSyncCurrentDelta = NAN;
  _tempSyncAverageDelta = NAN;

  _serial.println(F("OK TSYNC=STARTED PLACE_EXTERNAL_SENSOR_NEAR_NANO"));
}

void Command::stopTempSync() {
  _tempSyncActive = false;

  if (_tempSyncSamples == 0) {
    _serial.println(F("TSYNC RESULT=NO_VALID_SAMPLES"));
    return;
  }

  _serial.print(F("TSYNC RESULT=STOPPED AVG="));
  _serial.print(_tempSyncAverageDelta, 2);
  _serial.print(F(" SAMPLES="));
  _serial.print(_tempSyncSamples);
  _serial.print(F(" READY="));
  _serial.print(tempSyncReady() ? F("YES") : F("NO"));
  _serial.print(F(" RECOMMENDED_OFFSET="));
  _serial.println(tempSyncRecommendedOffset(), 2);
}

bool Command::tempSyncReady() const {
  return _tempSyncSamples >= TEMP_SYNC_MIN_SAMPLES;
}

float Command::tempSyncRecommendedOffset() const {
  if (_tempSyncSamples == 0 || isnan(_tempSyncAverageDelta)) {
    return NAN;
  }

  // The displayed Nano temperature already includes the configured offset.
  // Therefore the new absolute offset is current offset + measured difference.
  return INTERNAL_TEMP_CALIBRATION_OFFSET_C + _tempSyncAverageDelta;
}

void Command::sendStatus() {
  _serial.print(F("STATUS BAT="));
  _serial.print(_charger.batteryVoltage(), 2);

  _serial.print(F(" BTEMP="));
  if (_charger.temperatureValid()) {
    _serial.print(_charger.batteryTemperatureC(), 1);
  } else {
    _serial.print(F("INVALID"));
  }

  _serial.print(F(" NTEMP="));
  if (_charger.internalTemperatureValid()) {
    _serial.print(_charger.internalTemperatureC(), 1);
  } else {
    _serial.print(F("INVALID"));
  }

  _serial.print(F(" CHARGER="));
  _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));

  _serial.print(F(" STATE="));
  _serial.print(stateToken(_charger.state()));

  _serial.print(F(" MODE="));
  _serial.print(_charger.remoteInhibited() ? F("STOP") : F("AUTO"));

  sendTempSyncFields();
  _serial.println();
}

void Command::sendBattery() {
  _serial.print(F("BATTERY VOLTS="));
  _serial.println(_charger.batteryVoltage(), 2);
}

void Command::sendTemperature() {
  _serial.print(F("TEMP EXTERNAL="));
  if (_charger.temperatureValid()) {
    _serial.print(_charger.batteryTemperatureC(), 1);
  } else {
    _serial.print(F("INVALID"));
  }

  _serial.print(F(" NANO="));
  if (_charger.internalTemperatureValid()) {
    _serial.print(_charger.internalTemperatureC(), 1);
  } else {
    _serial.print(F("INVALID"));
  }

  _serial.print(F(" CHARGER="));
  _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE="));
  _serial.println(stateToken(_charger.state()));
}

void Command::sendState() {
  _serial.print(F("STATE CHARGER="));
  _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE="));
  _serial.print(stateToken(_charger.state()));
  _serial.print(F(" MODE="));
  _serial.println(_charger.remoteInhibited() ? F("STOP") : F("AUTO"));
}

void Command::sendTempSyncStatus() {
  _serial.print(F("TSYNC"));
  sendTempSyncFields();
  _serial.print(F(" EXT="));
  if (_charger.temperatureValid()) {
    _serial.print(_charger.batteryTemperatureC(), 1);
  } else {
    _serial.print(F("INVALID"));
  }
  _serial.print(F(" NANO="));
  if (_charger.internalTemperatureValid()) {
    _serial.print(_charger.internalTemperatureC(), 1);
  } else {
    _serial.print(F("INVALID"));
  }
  _serial.print(F(" CHARGER="));
  _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE="));
  _serial.println(stateToken(_charger.state()));
}

void Command::sendTempSyncFields() {
  _serial.print(F(" TSYNC="));
  _serial.print(_tempSyncActive ? F("ON") : F("OFF"));

  _serial.print(F(" TDELTA="));
  if (_tempSyncSamples > 0 && !isnan(_tempSyncCurrentDelta)) {
    _serial.print(_tempSyncCurrentDelta, 2);
  } else {
    _serial.print(F("INVALID"));
  }

  _serial.print(F(" TAVG="));
  if (_tempSyncSamples > 0 && !isnan(_tempSyncAverageDelta)) {
    _serial.print(_tempSyncAverageDelta, 2);
  } else {
    _serial.print(F("INVALID"));
  }

  _serial.print(F(" TSAMPLES="));
  _serial.print(_tempSyncSamples);

  _serial.print(F(" TREADY="));
  _serial.print(tempSyncReady() ? F("YES") : F("NO"));

  _serial.print(F(" TNEW="));
  const float newOffset = tempSyncRecommendedOffset();
  if (!isnan(newOffset)) {
    _serial.print(newOffset, 2);
  } else {
    _serial.print(F("INVALID"));
  }
}

void Command::sendHelp() {
  _serial.println(F("CMDS PING STATUS BATTERY TEMP STATE TSYNC_START TSYNC_STATUS TSYNC_STOP STOP AUTO HELP"));
}

void Command::sendError(const __FlashStringHelper* message) {
  _serial.print(F("ERR "));
  _serial.println(message);
}
