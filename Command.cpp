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
    _tempSyncPhase(TempSyncPhase::OFF),
    _tempSyncWasRemoteInhibited(false),
    _tempSyncLastSampleMs(0),
    _tempSyncBaselineSamples(0),
    _tempSyncBaselineSum(0.0f),
    _tempSyncBaselineAverage(NAN),
    _tempSyncChargeSamples(0),
    _tempSyncChargeSum(0.0f),
    _tempSyncChargeAverage(NAN),
    _tempSyncCurrentDelta(NAN),
    _tempSyncFinalAverage(NAN),
    _tempSyncRecommendedOffset(NAN) {
  _buffer[0] = '\0';
}

void Command::begin() {
  if (!ENABLE_ESP_COMMANDS) return;

  _serial.begin(ESP_COMMAND_BAUD);
  _serial.println(F("READY NANO_LEAD_ACID_CHARGER"));
}

void Command::update() {
  if (!ENABLE_ESP_COMMANDS) return;

  while (_serial.available() > 0) {
    handleChar(static_cast<char>(_serial.read()));
  }

  updateTempSync(millis());
}

void Command::handleChar(char c) {
  if (c == '\r') return;

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

  if (_discardUntilNewline) return;

  if (c == '\b' || c == 127) {
    if (_length > 0) {
      --_length;
      _buffer[_length] = '\0';
    }
    return;
  }

  if (!isprint(static_cast<unsigned char>(c))) return;

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
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) ++start;

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
      if (!previousWasSpace && out > 0U) normalized[out++] = ' ';
      previousWasSpace = true;
      continue;
    }

    normalized[out++] = static_cast<char>(toupper(raw));
    previousWasSpace = false;
  }

  if (out > 0U && normalized[out - 1U] == ' ') --out;

  normalized[out] = '\0';
  strncpy(_buffer, normalized, COMMAND_BUFFER_SIZE);
  _buffer[COMMAND_BUFFER_SIZE - 1U] = '\0';
}

void Command::processLine() {
  normalizeLine();
  if (_buffer[0] == '\0') return;

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
    stopTempSync(true);
    return;
  }

  if (strcmp(_buffer, "STOP") == 0 ||
      strcmp(_buffer, "OFF") == 0 ||
      strcmp(_buffer, "CHARGE OFF") == 0) {
    if (tempSyncActive()) stopTempSync(true);
    _charger.setRemoteInhibit(true);
    _serial.println(F("OK MODE=STOP CHARGER=OFF"));
    return;
  }

  if (strcmp(_buffer, "AUTO") == 0 || strcmp(_buffer, "CHARGE AUTO") == 0) {
    if (tempSyncActive()) {
      sendError(F("TSYNC_ACTIVE"));
      return;
    }
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

bool Command::tempSyncActive() const {
  return _tempSyncPhase == TempSyncPhase::BASELINE ||
         _tempSyncPhase == TempSyncPhase::WAIT_FOR_CHARGE ||
         _tempSyncPhase == TempSyncPhase::CHARGING;
}

bool Command::tempSyncReady() const {
  return _tempSyncPhase == TempSyncPhase::COMPLETE &&
         !isnan(_tempSyncRecommendedOffset);
}

const __FlashStringHelper* Command::tempSyncPhaseToken() const {
  switch (_tempSyncPhase) {
    case TempSyncPhase::OFF:             return F("OFF");
    case TempSyncPhase::BASELINE:        return F("BASELINE");
    case TempSyncPhase::WAIT_FOR_CHARGE: return F("WAIT_CHARGE");
    case TempSyncPhase::CHARGING:        return F("CHARGING");
    case TempSyncPhase::COMPLETE:        return F("COMPLETE");
    default:                             return F("UNKNOWN");
  }
}

void Command::startTempSync() {
  if (tempSyncActive()) {
    sendError(F("TSYNC_ALREADY_RUNNING"));
    return;
  }

  if (_charger.remoteInhibited()) {
    sendError(F("TSYNC_SET_AUTO_FIRST"));
    return;
  }

  if (!_charger.temperatureValid() || !_charger.internalTemperatureValid()) {
    sendError(F("TSYNC_SENSOR_INVALID"));
    return;
  }

  _tempSyncWasRemoteInhibited = _charger.remoteInhibited();
  _tempSyncPhase = TempSyncPhase::BASELINE;
  _tempSyncLastSampleMs = 0;

  _tempSyncBaselineSamples = 0;
  _tempSyncBaselineSum = 0.0f;
  _tempSyncBaselineAverage = NAN;

  _tempSyncChargeSamples = 0;
  _tempSyncChargeSum = 0.0f;
  _tempSyncChargeAverage = NAN;

  _tempSyncCurrentDelta = NAN;
  _tempSyncFinalAverage = NAN;
  _tempSyncRecommendedOffset = NAN;

  // Stage 1 must be a true charger-OFF baseline. This is an inhibit only;
  // it cannot force the charger ON later.
  _charger.setRemoteInhibit(true);

  _serial.print(F("OK TSYNC=STARTED PHASE=BASELINE TARGET="));
  _serial.println(TEMP_SYNC_SAMPLES_PER_PHASE);
}

void Command::updateTempSync(unsigned long nowMs) {
  if (!tempSyncActive()) return;

  // Stage transition: after baseline, release the temporary inhibit and wait
  // for the normal charger logic to actually enter CHARGING.
  if (_tempSyncPhase == TempSyncPhase::WAIT_FOR_CHARGE) {
    if (_charger.chargerEnabled() && _charger.state() == ChargerState::CHARGING) {
      _tempSyncPhase = TempSyncPhase::CHARGING;
      _tempSyncLastSampleMs = 0;
      _serial.println(F("TSYNC EVENT=CHARGING_DETECTED PHASE=CHARGING"));
    }
    return;
  }

  if ((nowMs - _tempSyncLastSampleMs) < TEMP_SYNC_SAMPLE_INTERVAL_MS) return;
  _tempSyncLastSampleMs = nowMs;

  if (!_charger.temperatureValid() || !_charger.internalTemperatureValid()) return;

  // Baseline samples are accepted only while the charger is definitely OFF.
  if (_tempSyncPhase == TempSyncPhase::BASELINE && _charger.chargerEnabled()) return;

  // Charging samples are accepted only while the Nano says it is really charging.
  if (_tempSyncPhase == TempSyncPhase::CHARGING &&
      (!_charger.chargerEnabled() || _charger.state() != ChargerState::CHARGING)) {
    return;
  }

  const float externalC = _charger.batteryTemperatureC();
  const float nanoC = _charger.internalTemperatureC();
  const float deltaC = externalC - nanoC;
  _tempSyncCurrentDelta = deltaC;

  if (_tempSyncPhase == TempSyncPhase::BASELINE) {
    _tempSyncBaselineSum += deltaC;
    ++_tempSyncBaselineSamples;
    _tempSyncBaselineAverage =
        _tempSyncBaselineSum / static_cast<float>(_tempSyncBaselineSamples);

    if (_tempSyncBaselineSamples >= TEMP_SYNC_SAMPLES_PER_PHASE) {
      _tempSyncPhase = TempSyncPhase::WAIT_FOR_CHARGE;

      // Return control to AUTO. The charger will turn on only if all existing
      // voltage and temperature rules say charging is allowed.
      _charger.setRemoteInhibit(false);

      _serial.print(F("TSYNC EVENT=BASELINE_DONE AVG="));
      _serial.print(_tempSyncBaselineAverage, 2);
      _serial.println(F(" ACTION=CHARGE_BATTERY_WAITING_FOR_CHARGER_ON"));
    }
    return;
  }

  if (_tempSyncPhase == TempSyncPhase::CHARGING) {
    _tempSyncChargeSum += deltaC;
    ++_tempSyncChargeSamples;
    _tempSyncChargeAverage =
        _tempSyncChargeSum / static_cast<float>(_tempSyncChargeSamples);

    if (_tempSyncChargeSamples >= TEMP_SYNC_SAMPLES_PER_PHASE) {
      completeTempSync();
    }
  }
}

void Command::completeTempSync() {
  if (_tempSyncBaselineSamples < TEMP_SYNC_SAMPLES_PER_PHASE ||
      _tempSyncChargeSamples < TEMP_SYNC_SAMPLES_PER_PHASE ||
      isnan(_tempSyncBaselineAverage) || isnan(_tempSyncChargeAverage)) {
    return;
  }

  // Equal 60/60 stages: average the OFF baseline correction and the real
  // charging correction. This gives one offset representing both conditions.
  _tempSyncFinalAverage =
      (_tempSyncBaselineAverage + _tempSyncChargeAverage) * 0.5f;
  _tempSyncRecommendedOffset =
      INTERNAL_TEMP_CALIBRATION_OFFSET_C + _tempSyncFinalAverage;

  _tempSyncPhase = TempSyncPhase::COMPLETE;

  // START is only allowed from AUTO, but preserve the original state anyway.
  _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);

  _serial.print(F("TSYNC RESULT=COMPLETE BASE="));
  _serial.print(_tempSyncBaselineAverage, 2);
  _serial.print(F(" CHARGE="));
  _serial.print(_tempSyncChargeAverage, 2);
  _serial.print(F(" FINAL="));
  _serial.print(_tempSyncFinalAverage, 2);
  _serial.print(F(" CHANGE_CODE_TO="));
  _serial.println(_tempSyncRecommendedOffset, 2);
}

void Command::stopTempSync(bool cancelled) {
  if (!tempSyncActive()) {
    if (_tempSyncPhase == TempSyncPhase::COMPLETE) {
      sendTempSyncStatus();
    } else {
      _serial.println(F("OK TSYNC=OFF"));
    }
    return;
  }

  _tempSyncPhase = TempSyncPhase::OFF;
  _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);

  _serial.println(cancelled ? F("OK TSYNC=CANCELLED") : F("OK TSYNC=STOPPED"));
}

void Command::sendStatus() {
  _serial.print(F("STATUS BAT="));
  _serial.print(_charger.batteryVoltage(), 2);

  _serial.print(F(" BTEMP="));
  if (_charger.temperatureValid()) _serial.print(_charger.batteryTemperatureC(), 1);
  else _serial.print(F("INVALID"));

  _serial.print(F(" NTEMP="));
  if (_charger.internalTemperatureValid()) _serial.print(_charger.internalTemperatureC(), 1);
  else _serial.print(F("INVALID"));

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
  if (_charger.temperatureValid()) _serial.print(_charger.batteryTemperatureC(), 1);
  else _serial.print(F("INVALID"));

  _serial.print(F(" NANO="));
  if (_charger.internalTemperatureValid()) _serial.print(_charger.internalTemperatureC(), 1);
  else _serial.print(F("INVALID"));

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
  _serial.print(F("TSYNC_STATUS"));
  sendTempSyncFields();
  _serial.print(F(" EXT="));
  if (_charger.temperatureValid()) _serial.print(_charger.batteryTemperatureC(), 1);
  else _serial.print(F("INVALID"));
  _serial.print(F(" NANO="));
  if (_charger.internalTemperatureValid()) _serial.print(_charger.internalTemperatureC(), 1);
  else _serial.print(F("INVALID"));
  _serial.print(F(" CHARGER="));
  _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE="));
  _serial.println(stateToken(_charger.state()));
}

void Command::sendTempSyncFields() {
  _serial.print(F(" TSYNC="));
  _serial.print(tempSyncActive() ? F("ON") : F("OFF"));

  _serial.print(F(" TPHASE="));
  _serial.print(tempSyncPhaseToken());

  _serial.print(F(" TDELTA="));
  if (!isnan(_tempSyncCurrentDelta)) _serial.print(_tempSyncCurrentDelta, 2);
  else _serial.print(F("INVALID"));

  _serial.print(F(" TBASE="));
  if (!isnan(_tempSyncBaselineAverage)) _serial.print(_tempSyncBaselineAverage, 2);
  else _serial.print(F("INVALID"));

  _serial.print(F(" TBSAMP="));
  _serial.print(_tempSyncBaselineSamples);

  _serial.print(F(" TCHG="));
  if (!isnan(_tempSyncChargeAverage)) _serial.print(_tempSyncChargeAverage, 2);
  else _serial.print(F("INVALID"));

  _serial.print(F(" TCSAMP="));
  _serial.print(_tempSyncChargeSamples);

  _serial.print(F(" TFINAL="));
  if (!isnan(_tempSyncFinalAverage)) _serial.print(_tempSyncFinalAverage, 2);
  else _serial.print(F("INVALID"));

  _serial.print(F(" TREADY="));
  _serial.print(tempSyncReady() ? F("YES") : F("NO"));

  _serial.print(F(" TNEW="));
  if (!isnan(_tempSyncRecommendedOffset)) _serial.print(_tempSyncRecommendedOffset, 2);
  else _serial.print(F("INVALID"));
}

void Command::sendHelp() {
  _serial.println(F("CMDS PING STATUS BATTERY TEMP STATE TSYNC_START TSYNC_STATUS TSYNC_STOP STOP AUTO HELP"));
}

void Command::sendError(const __FlashStringHelper* message) {
  _serial.print(F("ERR "));
  _serial.println(message);
}
