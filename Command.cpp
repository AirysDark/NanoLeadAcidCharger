#include "Command.h"
#include "ChargerController.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
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
}

Command::Command(ChargerController& charger)
  : _charger(charger),
    _serial(PIN_ESP_RX, PIN_ESP_TX),
    _length(0),
    _discardUntilNewline(false),
    _tempSyncPhase(TempSyncPhase::OFF),
    _tempSyncWasRemoteInhibited(false),
    _tempSyncLastSampleMs(0),
    _tempSyncCurrentDelta(NAN),
    _tempCalRaw(NAN),
    _tempCalC(NAN),
    _tempCountsPerC(NAN),
    _voltageCalPhase(VoltageCalPhase::OFF),
    _voltageCalSamples(0),
    _voltageCalRecommendedScale(NAN),
    _voltageCalRecommendedOffset(NAN) {
  _buffer[0] = '\0';
  for (uint8_t i = 0; i < 3; ++i) {
    _tempSamples[i] = 0;
    _tempExtSum[i] = 0.0f;
    _tempRawSum[i] = 0.0f;
    _tempExtAvg[i] = NAN;
    _tempRawAvg[i] = NAN;
  }
  for (uint8_t i = 0; i < VOLT_CAL_REQUIRED_SAMPLES; ++i) {
    _voltageCalNano[i] = NAN;
    _voltageCalActual[i] = NAN;
  }
}

void Command::begin() {
  if (!ENABLE_ESP_COMMANDS) return;
  _serial.begin(ESP_COMMAND_BAUD);
  _serial.println(F("READY NANO_LEAD_ACID_CHARGER"));
}

void Command::update() {
  if (!ENABLE_ESP_COMMANDS) return;
  while (_serial.available() > 0) handleChar(static_cast<char>(_serial.read()));
  updateTempSync(millis());
  updateVoltageCal();
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
    if (_length > 0) _buffer[--_length] = '\0';
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
  if (start != _buffer) memmove(_buffer, start, strlen(start) + 1U);

  size_t len = strlen(_buffer);
  while (len > 0U && isspace(static_cast<unsigned char>(_buffer[len - 1U]))) _buffer[--len] = '\0';

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

  if (strcmp(_buffer, "PING") == 0) { _serial.println(F("PONG")); return; }
  if (strcmp(_buffer, "STATUS") == 0 || strcmp(_buffer, "GET STATUS") == 0) { sendStatus(); return; }
  if (strcmp(_buffer, "BATTERY") == 0 || strcmp(_buffer, "GET BATTERY") == 0) { sendBattery(); return; }
  if (strcmp(_buffer, "TEMP") == 0 || strcmp(_buffer, "TEMPERATURE") == 0 || strcmp(_buffer, "GET TEMP") == 0) { sendTemperature(); return; }
  if (strcmp(_buffer, "STATE") == 0 || strcmp(_buffer, "GET STATE") == 0) { sendState(); return; }

  if (strcmp(_buffer, "TSYNC START") == 0 || strcmp(_buffer, "TEMP SYNC START") == 0) { startTempSync(); return; }
  if (strcmp(_buffer, "TSYNC STATUS") == 0 || strcmp(_buffer, "TEMP SYNC STATUS") == 0) { sendTempSyncStatus(); return; }
  if (strcmp(_buffer, "TSYNC STOP") == 0 || strcmp(_buffer, "TEMP SYNC STOP") == 0) { stopTempSync(true); return; }

  if (strcmp(_buffer, "VCAL START") == 0 || strcmp(_buffer, "VOLT CAL START") == 0) { startVoltageCal(); return; }
  if (strncmp(_buffer, "VCAL SAMPLE ", 12) == 0) {
    const char* valueText = _buffer + 12;
    char* endPtr = nullptr;
    const float actualVolts = static_cast<float>(strtod(valueText, &endPtr));
    if (endPtr == valueText || *endPtr != '\0' || isnan(actualVolts)) { sendError(F("VCAL_BAD_NUMBER")); return; }
    addVoltageCalSample(actualVolts);
    return;
  }
  if (strcmp(_buffer, "VCAL STATUS") == 0 || strcmp(_buffer, "VOLT CAL STATUS") == 0) { sendVoltageCalStatus(); return; }
  if (strcmp(_buffer, "VCAL STOP") == 0 || strcmp(_buffer, "VOLT CAL STOP") == 0) { stopVoltageCal(); return; }

  if (strcmp(_buffer, "STOP") == 0 || strcmp(_buffer, "OFF") == 0 || strcmp(_buffer, "CHARGE OFF") == 0) {
    if (tempSyncActive()) stopTempSync(true);
    _charger.setRemoteInhibit(true);
    _serial.println(F("OK MODE=STOP CHARGER=OFF"));
    return;
  }

  if (strcmp(_buffer, "AUTO") == 0 || strcmp(_buffer, "CHARGE AUTO") == 0) {
    if (tempSyncActive()) { sendError(F("TSYNC_ACTIVE")); return; }
    _charger.setRemoteInhibit(false);
    _serial.println(F("OK MODE=AUTO"));
    return;
  }

  if (strcmp(_buffer, "HELP") == 0 || strcmp(_buffer, "?") == 0) { sendHelp(); return; }
  sendError(F("UNKNOWN_COMMAND"));
}

bool Command::tempSyncActive() const {
  return _tempSyncPhase == TempSyncPhase::POINT1 ||
         _tempSyncPhase == TempSyncPhase::WAIT_POINT2 ||
         _tempSyncPhase == TempSyncPhase::POINT2 ||
         _tempSyncPhase == TempSyncPhase::WAIT_POINT3 ||
         _tempSyncPhase == TempSyncPhase::POINT3;
}

bool Command::tempSyncReady() const {
  return _tempSyncPhase == TempSyncPhase::COMPLETE &&
         !isnan(_tempCalRaw) && !isnan(_tempCalC) && !isnan(_tempCountsPerC);
}

const __FlashStringHelper* Command::tempSyncPhaseToken() const {
  switch (_tempSyncPhase) {
    case TempSyncPhase::OFF:         return F("OFF");
    case TempSyncPhase::POINT1:      return F("POINT1");
    case TempSyncPhase::WAIT_POINT2: return F("WAIT_POINT2");
    case TempSyncPhase::POINT2:      return F("POINT2");
    case TempSyncPhase::WAIT_POINT3: return F("WAIT_POINT3");
    case TempSyncPhase::POINT3:      return F("POINT3");
    case TempSyncPhase::COMPLETE:    return F("COMPLETE");
    default:                         return F("UNKNOWN");
  }
}

void Command::startTempSync() {
  if (tempSyncActive()) { sendError(F("TSYNC_ALREADY_RUNNING")); return; }
  if (voltageCalActive()) { sendError(F("VCAL_ACTIVE")); return; }
  if (_charger.remoteInhibited()) { sendError(F("TSYNC_SET_AUTO_FIRST")); return; }
  const uint16_t internalRaw = _charger.internalTemperatureRawAdc();
  const bool internalRawValid =
      internalRaw >= INTERNAL_TEMP_RAW_MIN_VALID &&
      internalRaw <= INTERNAL_TEMP_RAW_MAX_VALID;
  // TEMP SYNC fits external reference temperature directly against the raw
  // internal ADC value. It must therefore be allowed to run even when the old
  // Celsius calibration itself is currently INVALID.
  if (!_charger.temperatureValid() || !internalRawValid) {
    sendError(F("TSYNC_SENSOR_INVALID"));
    return;
  }

  _tempSyncWasRemoteInhibited = _charger.remoteInhibited();
  _tempSyncPhase = TempSyncPhase::POINT1;
  _tempSyncLastSampleMs = 0;
  _tempSyncCurrentDelta = NAN;
  _tempCalRaw = NAN;
  _tempCalC = NAN;
  _tempCountsPerC = NAN;
  for (uint8_t i = 0; i < 3; ++i) {
    _tempSamples[i] = 0;
    _tempExtSum[i] = 0.0f;
    _tempRawSum[i] = 0.0f;
    _tempExtAvg[i] = NAN;
    _tempRawAvg[i] = NAN;
  }

  _charger.setRemoteInhibit(true);
  _serial.print(F("OK TSYNC=STARTED PHASE=POINT1 TARGET="));
  _serial.println(TEMP_SYNC_SAMPLES_PER_POINT);
}

void Command::addTempSample(uint8_t pointIndex) {
  const float extC = _charger.batteryTemperatureC();
  const float nanoC = _charger.internalTemperatureC();
  const float raw = static_cast<float>(_charger.internalTemperatureRawAdc());

  // The raw ADC value is what the fit needs. If the old internal calibration
  // is invalid, keep collecting calibration samples and simply omit the live
  // external-minus-Nano delta until a valid Celsius value exists.
  _tempSyncCurrentDelta =
      _charger.internalTemperatureValid() ? (extC - nanoC) : NAN;
  _tempExtSum[pointIndex] += extC;
  _tempRawSum[pointIndex] += raw;
  ++_tempSamples[pointIndex];
  _tempExtAvg[pointIndex] = _tempExtSum[pointIndex] / static_cast<float>(_tempSamples[pointIndex]);
  _tempRawAvg[pointIndex] = _tempRawSum[pointIndex] / static_cast<float>(_tempSamples[pointIndex]);
}

void Command::updateTempSync(unsigned long nowMs) {
  if (!tempSyncActive()) return;

  const uint16_t internalRaw = _charger.internalTemperatureRawAdc();
  const bool internalRawValid =
      internalRaw >= INTERNAL_TEMP_RAW_MIN_VALID &&
      internalRaw <= INTERNAL_TEMP_RAW_MAX_VALID;
  if (!_charger.temperatureValid() || !internalRawValid) return;

  const float extC = _charger.batteryTemperatureC();

  if (_tempSyncPhase == TempSyncPhase::WAIT_POINT2) {
    if (_charger.chargerEnabled() &&
        _charger.state() == ChargerState::CHARGING &&
        extC >= (_tempExtAvg[0] + TEMP_SYNC_MIN_RISE_C)) {
      _tempSyncPhase = TempSyncPhase::POINT2;
      _tempSyncLastSampleMs = 0;
      _serial.println(F("TSYNC EVENT=POINT2_READY PHASE=POINT2"));
    }
    return;
  }

  if (_tempSyncPhase == TempSyncPhase::WAIT_POINT3) {
    if (_charger.chargerEnabled() &&
        _charger.state() == ChargerState::CHARGING &&
        extC >= (_tempExtAvg[1] + TEMP_SYNC_MIN_RISE_C)) {
      _tempSyncPhase = TempSyncPhase::POINT3;
      _tempSyncLastSampleMs = 0;
      _serial.println(F("TSYNC EVENT=POINT3_READY PHASE=POINT3"));
    }
    return;
  }

  if ((nowMs - _tempSyncLastSampleMs) < TEMP_SYNC_SAMPLE_INTERVAL_MS) return;
  _tempSyncLastSampleMs = nowMs;

  if (_tempSyncPhase == TempSyncPhase::POINT1) {
    if (_charger.chargerEnabled()) return;
    addTempSample(0);
    if (_tempSamples[0] >= TEMP_SYNC_SAMPLES_PER_POINT) {
      _tempSyncPhase = TempSyncPhase::WAIT_POINT2;
      _charger.setRemoteInhibit(false);
      _serial.print(F("TSYNC EVENT=POINT1_DONE EXT="));
      _serial.print(_tempExtAvg[0], 2);
      _serial.println(F(" ACTION=CHARGE_AND_WAIT_FOR_TEMP_RISE"));
    }
    return;
  }

  if (_tempSyncPhase == TempSyncPhase::POINT2) {
    if (!_charger.chargerEnabled() || _charger.state() != ChargerState::CHARGING) return;
    addTempSample(1);
    if (_tempSamples[1] >= TEMP_SYNC_SAMPLES_PER_POINT) {
      _tempSyncPhase = TempSyncPhase::WAIT_POINT3;
      _serial.print(F("TSYNC EVENT=POINT2_DONE EXT="));
      _serial.print(_tempExtAvg[1], 2);
      _serial.println(F(" ACTION=KEEP_CHARGING_WAIT_FOR_MORE_TEMP_RISE"));
    }
    return;
  }

  if (_tempSyncPhase == TempSyncPhase::POINT3) {
    if (!_charger.chargerEnabled() || _charger.state() != ChargerState::CHARGING) return;
    addTempSample(2);
    if (_tempSamples[2] >= TEMP_SYNC_SAMPLES_PER_POINT) completeTempSync();
  }
}

void Command::completeTempSync() {
  if (_tempSamples[0] < TEMP_SYNC_SAMPLES_PER_POINT ||
      _tempSamples[1] < TEMP_SYNC_SAMPLES_PER_POINT ||
      _tempSamples[2] < TEMP_SYNC_SAMPLES_PER_POINT) return;

  float minTemp = _tempExtAvg[0];
  float maxTemp = _tempExtAvg[0];
  float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f;

  for (uint8_t i = 0; i < 3; ++i) {
    const float x = _tempRawAvg[i];
    const float y = _tempExtAvg[i];
    if (y < minTemp) minTemp = y;
    if (y > maxTemp) maxTemp = y;
    sumX += x;
    sumY += y;
    sumXX += x * x;
    sumXY += x * y;
  }

  if ((maxTemp - minTemp) < TEMP_SYNC_MIN_TOTAL_SPAN_C) {
    _tempSyncPhase = TempSyncPhase::OFF;
    _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);
    sendError(F("TSYNC_TEMP_SPAN_TOO_SMALL_RESTART"));
    return;
  }

  const float n = 3.0f;
  const float denominator = (n * sumXX) - (sumX * sumX);
  if (fabs(denominator) < 0.000001f) {
    _tempSyncPhase = TempSyncPhase::OFF;
    _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);
    sendError(F("TSYNC_FIT_FAILED_RESTART"));
    return;
  }

  const float slopeCPerCount = ((n * sumXY) - (sumX * sumY)) / denominator;
  const float interceptC = (sumY - (slopeCPerCount * sumX)) / n;
  if (fabs(slopeCPerCount) < 0.000001f) {
    _tempSyncPhase = TempSyncPhase::OFF;
    _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);
    sendError(F("TSYNC_SLOPE_INVALID_RESTART"));
    return;
  }

  _tempCalRaw = sumX / n;
  _tempCalC = (slopeCPerCount * _tempCalRaw) + interceptC;
  _tempCountsPerC = 1.0f / slopeCPerCount;
  _tempSyncPhase = TempSyncPhase::COMPLETE;
  _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);

  _serial.print(F("TSYNC RESULT=COMPLETE CALRAW="));
  _serial.print(_tempCalRaw, 2);
  _serial.print(F(" CALC="));
  _serial.print(_tempCalC, 2);
  _serial.print(F(" COUNTS="));
  _serial.println(_tempCountsPerC, 4);
}

void Command::stopTempSync(bool cancelled) {
  if (!tempSyncActive()) {
    if (_tempSyncPhase == TempSyncPhase::COMPLETE) sendTempSyncStatus();
    else _serial.println(F("OK TSYNC=OFF"));
    return;
  }
  _tempSyncPhase = TempSyncPhase::OFF;
  _charger.setRemoteInhibit(_tempSyncWasRemoteInhibited);
  _serial.println(cancelled ? F("OK TSYNC=CANCELLED") : F("OK TSYNC=STOPPED"));
}

bool Command::voltageCalActive() const {
  return _voltageCalPhase == VoltageCalPhase::WAIT_INPUT_1 ||
         _voltageCalPhase == VoltageCalPhase::WAIT_RISE_2 ||
         _voltageCalPhase == VoltageCalPhase::WAIT_INPUT_2 ||
         _voltageCalPhase == VoltageCalPhase::WAIT_RISE_3 ||
         _voltageCalPhase == VoltageCalPhase::WAIT_INPUT_3;
}

bool Command::voltageCalReady() const {
  return _voltageCalPhase == VoltageCalPhase::COMPLETE &&
         !isnan(_voltageCalRecommendedScale) && !isnan(_voltageCalRecommendedOffset);
}

const __FlashStringHelper* Command::voltageCalPhaseToken() const {
  switch (_voltageCalPhase) {
    case VoltageCalPhase::OFF:          return F("OFF");
    case VoltageCalPhase::WAIT_INPUT_1: return F("INPUT1");
    case VoltageCalPhase::WAIT_RISE_2:  return F("WAIT_RISE2");
    case VoltageCalPhase::WAIT_INPUT_2: return F("INPUT2");
    case VoltageCalPhase::WAIT_RISE_3:  return F("WAIT_RISE3");
    case VoltageCalPhase::WAIT_INPUT_3: return F("INPUT3");
    case VoltageCalPhase::COMPLETE:     return F("COMPLETE");
    default:                            return F("UNKNOWN");
  }
}

void Command::startVoltageCal() {
  if (voltageCalActive()) { sendError(F("VCAL_ALREADY_RUNNING")); return; }
  if (tempSyncActive()) { sendError(F("TSYNC_ACTIVE")); return; }

  const float nanoVolts = _charger.batteryVoltage();
  if (isnan(nanoVolts) || nanoVolts < MIN_VALID_BATTERY_VOLTS || nanoVolts > MAX_VALID_BATTERY_VOLTS) {
    sendError(F("VCAL_BATTERY_INVALID"));
    return;
  }

  _voltageCalPhase = VoltageCalPhase::WAIT_INPUT_1;
  _voltageCalSamples = 0;
  _voltageCalRecommendedScale = NAN;
  _voltageCalRecommendedOffset = NAN;
  for (uint8_t i = 0; i < VOLT_CAL_REQUIRED_SAMPLES; ++i) {
    _voltageCalNano[i] = NAN;
    _voltageCalActual[i] = NAN;
  }
  _serial.println(F("OK VCAL=STARTED PHASE=INPUT1 ACTION=ENTER_MULTIMETER_VOLTS"));
}

void Command::updateVoltageCal() {
  if (_voltageCalSamples == 0) return;
  const float currentNano = _charger.batteryVoltage();
  const float previousNano = _voltageCalNano[_voltageCalSamples - 1U];
  if (isnan(currentNano) || isnan(previousNano)) return;

  if (_voltageCalPhase == VoltageCalPhase::WAIT_RISE_2 && currentNano >= (previousNano + VOLT_CAL_MIN_STEP_VOLTS)) {
    _voltageCalPhase = VoltageCalPhase::WAIT_INPUT_2;
    _serial.println(F("VCAL EVENT=RISE_DETECTED PHASE=INPUT2 ACTION=ENTER_MULTIMETER_VOLTS"));
    return;
  }
  if (_voltageCalPhase == VoltageCalPhase::WAIT_RISE_3 && currentNano >= (previousNano + VOLT_CAL_MIN_STEP_VOLTS)) {
    _voltageCalPhase = VoltageCalPhase::WAIT_INPUT_3;
    _serial.println(F("VCAL EVENT=RISE_DETECTED PHASE=INPUT3 ACTION=ENTER_MULTIMETER_VOLTS"));
  }
}

void Command::addVoltageCalSample(float actualVolts) {
  const bool waitingForInput =
      _voltageCalPhase == VoltageCalPhase::WAIT_INPUT_1 ||
      _voltageCalPhase == VoltageCalPhase::WAIT_INPUT_2 ||
      _voltageCalPhase == VoltageCalPhase::WAIT_INPUT_3;
  if (!waitingForInput) { sendError(F("VCAL_NOT_READY_FOR_INPUT")); return; }
  if (actualVolts < MIN_VALID_BATTERY_VOLTS || actualVolts > MAX_VALID_BATTERY_VOLTS || isnan(actualVolts)) {
    sendError(F("VCAL_ACTUAL_OUT_OF_RANGE")); return;
  }

  const float nanoVolts = _charger.batteryVoltage();
  if (nanoVolts < MIN_VALID_BATTERY_VOLTS || nanoVolts > MAX_VALID_BATTERY_VOLTS || isnan(nanoVolts)) {
    sendError(F("VCAL_NANO_READING_INVALID")); return;
  }
  if (_voltageCalSamples > 0) {
    const float previousNano = _voltageCalNano[_voltageCalSamples - 1U];
    if (nanoVolts < (previousNano + VOLT_CAL_MIN_STEP_VOLTS)) { sendError(F("VCAL_WAIT_FOR_MORE_RISE")); return; }
  }
  if (_voltageCalSamples >= VOLT_CAL_REQUIRED_SAMPLES) { sendError(F("VCAL_ALREADY_COMPLETE")); return; }

  _voltageCalNano[_voltageCalSamples] = nanoVolts;
  _voltageCalActual[_voltageCalSamples] = actualVolts;
  ++_voltageCalSamples;

  _serial.print(F("VCAL SAMPLE="));
  _serial.print(_voltageCalSamples);
  _serial.print(F(" NANO="));
  _serial.print(nanoVolts, 3);
  _serial.print(F(" ACTUAL="));
  _serial.println(actualVolts, 3);

  if (_voltageCalSamples == 1U) { _voltageCalPhase = VoltageCalPhase::WAIT_RISE_2; return; }
  if (_voltageCalSamples == 2U) { _voltageCalPhase = VoltageCalPhase::WAIT_RISE_3; return; }
  completeVoltageCal();
}

void Command::completeVoltageCal() {
  if (_voltageCalSamples != VOLT_CAL_REQUIRED_SAMPLES) return;

  float minNano = _voltageCalNano[0];
  float maxNano = _voltageCalNano[0];
  float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f;
  for (uint8_t i = 0; i < VOLT_CAL_REQUIRED_SAMPLES; ++i) {
    const float x = _voltageCalNano[i];
    const float y = _voltageCalActual[i];
    if (x < minNano) minNano = x;
    if (x > maxNano) maxNano = x;
    sumX += x; sumY += y; sumXX += x * x; sumXY += x * y;
  }

  if ((maxNano - minNano) < VOLT_CAL_MIN_TOTAL_SPAN_VOLTS) {
    _voltageCalPhase = VoltageCalPhase::OFF;
    sendError(F("VCAL_SPAN_TOO_SMALL_RESTART"));
    return;
  }

  const float n = static_cast<float>(VOLT_CAL_REQUIRED_SAMPLES);
  const float denominator = (n * sumXX) - (sumX * sumX);
  if (fabs(denominator) < 0.000001f) {
    _voltageCalPhase = VoltageCalPhase::OFF;
    sendError(F("VCAL_FIT_FAILED_RESTART"));
    return;
  }

  const float correctionSlope = ((n * sumXY) - (sumX * sumY)) / denominator;
  const float correctionIntercept = (sumY - (correctionSlope * sumX)) / n;
  _voltageCalRecommendedScale = correctionSlope * BATTERY_VOLTAGE_CALIBRATION;
  _voltageCalRecommendedOffset = (correctionSlope * BATTERY_VOLTAGE_OFFSET_VOLTS) + correctionIntercept;
  _voltageCalPhase = VoltageCalPhase::COMPLETE;

  _serial.print(F("VCAL RESULT=COMPLETE SCALE="));
  _serial.print(_voltageCalRecommendedScale, 6);
  _serial.print(F(" OFFSET="));
  _serial.println(_voltageCalRecommendedOffset, 4);
}

void Command::stopVoltageCal() {
  if (!voltageCalActive()) {
    if (_voltageCalPhase == VoltageCalPhase::COMPLETE) sendVoltageCalStatus();
    else _serial.println(F("OK VCAL=OFF"));
    return;
  }
  _voltageCalPhase = VoltageCalPhase::OFF;
  _serial.println(F("OK VCAL=CANCELLED"));
}

void Command::sendStatus() {
  _serial.print(F("STATUS BAT=")); _serial.print(_charger.batteryVoltage(), 2);
  _serial.print(F(" BTEMP=")); if (_charger.temperatureValid()) _serial.print(_charger.batteryTemperatureC(), 1); else _serial.print(F("INVALID"));
  _serial.print(F(" NTEMP=")); if (_charger.internalTemperatureValid()) _serial.print(_charger.internalTemperatureC(), 1); else _serial.print(F("INVALID"));
  _serial.print(F(" NRAW=")); _serial.print(_charger.internalTemperatureRawAdc());
  _serial.print(F(" CHARGER=")); _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE=")); _serial.print(stateToken(_charger.state()));
  _serial.print(F(" MODE=")); _serial.print(_charger.remoteInhibited() ? F("STOP") : F("AUTO"));
  sendTempSyncFields();
  sendVoltageCalFields();
  _serial.println();
}

void Command::sendBattery() {
  _serial.print(F("BATTERY VOLTS="));
  _serial.println(_charger.batteryVoltage(), 2);
}

void Command::sendTemperature() {
  _serial.print(F("TEMP EXTERNAL=")); if (_charger.temperatureValid()) _serial.print(_charger.batteryTemperatureC(), 1); else _serial.print(F("INVALID"));
  _serial.print(F(" NANO=")); if (_charger.internalTemperatureValid()) _serial.print(_charger.internalTemperatureC(), 1); else _serial.print(F("INVALID"));
  _serial.print(F(" RAW=")); _serial.print(_charger.internalTemperatureRawAdc());
  _serial.print(F(" CHARGER=")); _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE=")); _serial.println(stateToken(_charger.state()));
}

void Command::sendState() {
  _serial.print(F("STATE CHARGER=")); _serial.print(_charger.chargerEnabled() ? F("ON") : F("OFF"));
  _serial.print(F(" STATE=")); _serial.print(stateToken(_charger.state()));
  _serial.print(F(" MODE=")); _serial.println(_charger.remoteInhibited() ? F("STOP") : F("AUTO"));
}

void Command::sendTempSyncStatus() {
  _serial.print(F("TSYNC_STATUS"));
  sendTempSyncFields();
  _serial.println();
}

void Command::sendTempSyncFields() {
  _serial.print(F(" TSYNC=")); _serial.print(tempSyncActive() ? F("ON") : F("OFF"));
  _serial.print(F(" TPHASE=")); _serial.print(tempSyncPhaseToken());
  _serial.print(F(" TDELTA=")); if (!isnan(_tempSyncCurrentDelta)) _serial.print(_tempSyncCurrentDelta, 2); else _serial.print(F("INVALID"));

  for (uint8_t i = 0; i < 3; ++i) {
    _serial.print(F(" T")); _serial.print(i + 1); _serial.print(F("EXT="));
    if (!isnan(_tempExtAvg[i])) _serial.print(_tempExtAvg[i], 2); else _serial.print(F("INVALID"));
    _serial.print(F(" T")); _serial.print(i + 1); _serial.print(F("RAW="));
    if (!isnan(_tempRawAvg[i])) _serial.print(_tempRawAvg[i], 2); else _serial.print(F("INVALID"));
    _serial.print(F(" T")); _serial.print(i + 1); _serial.print(F("S=")); _serial.print(_tempSamples[i]);
  }

  _serial.print(F(" TREADY=")); _serial.print(tempSyncReady() ? F("YES") : F("NO"));
  _serial.print(F(" TCALRAW=")); if (!isnan(_tempCalRaw)) _serial.print(_tempCalRaw, 2); else _serial.print(F("INVALID"));
  _serial.print(F(" TCALC=")); if (!isnan(_tempCalC)) _serial.print(_tempCalC, 2); else _serial.print(F("INVALID"));
  _serial.print(F(" TCOUNTS=")); if (!isnan(_tempCountsPerC)) _serial.print(_tempCountsPerC, 4); else _serial.print(F("INVALID"));
}

void Command::sendVoltageCalStatus() {
  _serial.print(F("VCAL_STATUS"));
  sendVoltageCalFields();
  _serial.println();
}

void Command::sendVoltageCalFields() {
  _serial.print(F(" VCAL=")); _serial.print(voltageCalActive() ? F("ON") : F("OFF"));
  _serial.print(F(" VPHASE=")); _serial.print(voltageCalPhaseToken());
  _serial.print(F(" VSAMP=")); _serial.print(_voltageCalSamples);
  _serial.print(F(" VTARGET="));
  if ((_voltageCalPhase == VoltageCalPhase::WAIT_RISE_2 || _voltageCalPhase == VoltageCalPhase::WAIT_RISE_3) && _voltageCalSamples > 0) {
    _serial.print(_voltageCalNano[_voltageCalSamples - 1U] + VOLT_CAL_MIN_STEP_VOLTS, 3);
  } else _serial.print(F("INVALID"));
  _serial.print(F(" VREADY=")); _serial.print(voltageCalReady() ? F("YES") : F("NO"));
  _serial.print(F(" VSCALE=")); if (!isnan(_voltageCalRecommendedScale)) _serial.print(_voltageCalRecommendedScale, 6); else _serial.print(F("INVALID"));
  _serial.print(F(" VOFF=")); if (!isnan(_voltageCalRecommendedOffset)) _serial.print(_voltageCalRecommendedOffset, 4); else _serial.print(F("INVALID"));
}

void Command::sendHelp() {
  _serial.println(F("CMDS PING STATUS BATTERY TEMP STATE TSYNC_START TSYNC_STATUS TSYNC_STOP VCAL_START VCAL_SAMPLE_xx.xx VCAL_STATUS VCAL_STOP STOP AUTO HELP"));
}

void Command::sendError(const __FlashStringHelper* message) {
  _serial.print(F("ERR "));
  _serial.println(message);
}
