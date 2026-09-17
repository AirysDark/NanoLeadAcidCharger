#include "ChargerController.h"
#include "PinsAndConfig.h"
#include "Debug.h"

ChargerController::ChargerController()
  : _temperature(PIN_TEMP_SENSOR),
    _internalTemperature(),
    _gateway(),
    _state(ChargerState::STARTUP),
    _temperatureLockout(false),
    _internalTemperatureLockout(false),
    _remoteInhibit(false),
    _batteryVoltage(0.0f),
    _lastControlMs(0),
    _lastDebugMs(0),
    _lastTurnedOffMs(0) {
}

void ChargerController::begin() {
  _gateway.begin();

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  analogReference(DEFAULT);

  Debug::begin();
  _temperature.begin();
  if (ENABLE_INTERNAL_TEMP_MONITOR) {
    _internalTemperature.begin();
  }

  _remoteInhibit = false;
  _batteryVoltage = readBatteryVoltage();
  _lastTurnedOffMs = millis();

  setCharger(false, ChargerState::STARTUP, millis());
  Debug::printStartup(ENABLE_INTERNAL_TEMP_MONITOR);
}

void ChargerController::update() {
  const unsigned long nowMs = millis();

  _temperature.update(nowMs);
  if (ENABLE_INTERNAL_TEMP_MONITOR) {
    _internalTemperature.update(nowMs);
  }

  if ((nowMs - _lastControlMs) >= CONTROL_INTERVAL_MS) {
    _lastControlMs = nowMs;
    _batteryVoltage = readBatteryVoltage();
    runControl(nowMs);
  }

  if (ENABLE_DEBUG && (nowMs - _lastDebugMs) >= DEBUG_INTERVAL_MS) {
    _lastDebugMs = nowMs;
    const unsigned long offForMs = _gateway.enabled() ? 0UL : (nowMs - _lastTurnedOffMs);
    Debug::printStatus(_batteryVoltage,
                       _temperature.valid(),
                       _temperature.celsius(),
                       ENABLE_INTERNAL_TEMP_MONITOR,
                       _internalTemperature.valid(),
                       _internalTemperature.celsius(),
                       _internalTemperature.rawAdc(),
                       _gateway.enabled(),
                       _state,
                       offForMs);
  }
}

float ChargerController::readBatteryVoltage() {
  (void)analogRead(PIN_BATTERY_VOLTAGE);

  uint32_t total = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    total += analogRead(PIN_BATTERY_VOLTAGE);
  }

  const float averageCounts = static_cast<float>(total) / ADC_SAMPLES;
  const float adcVolts = averageCounts * (ADC_REFERENCE_VOLTS / ADC_MAX_COUNTS);
  const float dividerRatio =
      (BATTERY_DIVIDER_R_TOP_OHMS + BATTERY_DIVIDER_R_BOTTOM_OHMS) /
      BATTERY_DIVIDER_R_BOTTOM_OHMS;

  const float nominalBatteryVolts = adcVolts * dividerRatio;
  return (nominalBatteryVolts * BATTERY_VOLTAGE_CALIBRATION) +
         BATTERY_VOLTAGE_OFFSET_VOLTS;
}

void ChargerController::runControl(unsigned long nowMs) {
  if (_batteryVoltage < MIN_VALID_BATTERY_VOLTS ||
      _batteryVoltage > MAX_VALID_BATTERY_VOLTS ||
      isnan(_batteryVoltage)) {
    setCharger(false, ChargerState::VOLTAGE_FAULT, nowMs);
    return;
  }

  if (_batteryVoltage >= HARD_OVERVOLTAGE_VOLTS) {
    setCharger(false, ChargerState::OVERVOLTAGE_FAULT, nowMs);
    return;
  }

  if (REQUIRE_TEMP_SENSOR && !_temperature.valid()) {
    setCharger(false, ChargerState::SENSOR_FAULT, nowMs);
    return;
  }

  if (ENABLE_INTERNAL_TEMP_MONITOR &&
      REQUIRE_INTERNAL_TEMP_MONITOR &&
      !_internalTemperature.valid()) {
    setCharger(false, ChargerState::INTERNAL_TEMP_FAULT, nowMs);
    return;
  }

  if (_temperature.valid()) {
    if (_temperature.celsius() >= TEMP_CUTOFF_C) {
      _temperatureLockout = true;
    } else if (_temperature.celsius() <= TEMP_RESTART_C) {
      _temperatureLockout = false;
    }
  }

  if (_temperatureLockout) {
    setCharger(false, ChargerState::BATTERY_TEMP_OFF, nowMs);
    return;
  }

  if (ENABLE_INTERNAL_TEMP_MONITOR && _internalTemperature.valid()) {
    if (_internalTemperature.celsius() >= INTERNAL_TEMP_CUTOFF_C) {
      _internalTemperatureLockout = true;
    } else if (_internalTemperature.celsius() <= INTERNAL_TEMP_RESTART_C) {
      _internalTemperatureLockout = false;
    }
  }

  if (_internalTemperatureLockout) {
    setCharger(false, ChargerState::INTERNAL_TEMP_OFF, nowMs);
    return;
  }

  if (_remoteInhibit) {
    setCharger(false, ChargerState::REMOTE_OFF, nowMs);
    return;
  }

  if (_gateway.enabled() && _batteryVoltage >= CHARGE_CUTOFF_VOLTS) {
    setCharger(false, ChargerState::FULL_OFF, nowMs);
    return;
  }

  if (_gateway.enabled()) {
    _state = ChargerState::CHARGING;
    return;
  }

  const bool offTimeExpired = (nowMs - _lastTurnedOffMs) >= MIN_OFF_TIME_MS;
  if (_batteryVoltage <= CHARGE_RESTART_VOLTS && offTimeExpired) {
    setCharger(true, ChargerState::CHARGING, nowMs);
  } else {
    if (_state == ChargerState::STARTUP ||
        _state == ChargerState::CHARGING ||
        _state == ChargerState::REMOTE_OFF) {
      setCharger(false, ChargerState::FULL_OFF, nowMs);
    }
  }
}

void ChargerController::setCharger(bool enabled,
                                   ChargerState newState,
                                   unsigned long nowMs) {
  const bool outputChanged = (_gateway.enabled() != enabled);
  const ChargerState oldState = _state;

  if (outputChanged) {
    _gateway.setEnabled(enabled);

    if (!enabled) {
      _lastTurnedOffMs = nowMs;
    }
  } else {
    _gateway.reassert();
  }

  _state = newState;
  digitalWrite(PIN_STATUS_LED, enabled ? HIGH : LOW);

  if (outputChanged || oldState != newState) {
    Debug::printChargerChange(enabled, newState, _batteryVoltage);
  }
}

void ChargerController::setRemoteInhibit(bool inhibit) {
  _remoteInhibit = inhibit;

  if (_remoteInhibit) {
    setCharger(false, ChargerState::REMOTE_OFF, millis());
  }
}

bool ChargerController::remoteInhibited() const {
  return _remoteInhibit;
}

float ChargerController::batteryVoltage() const {
  return _batteryVoltage;
}

float ChargerController::batteryTemperatureC() const {
  return _temperature.celsius();
}

float ChargerController::internalTemperatureC() const {
  return _internalTemperature.celsius();
}

uint16_t ChargerController::internalTemperatureRawAdc() const {
  return _internalTemperature.rawAdc();
}

bool ChargerController::temperatureValid() const {
  return _temperature.valid();
}

bool ChargerController::internalTemperatureValid() const {
  return _internalTemperature.valid();
}

bool ChargerController::chargerEnabled() const {
  return _gateway.enabled();
}

ChargerState ChargerController::state() const {
  return _state;
}
