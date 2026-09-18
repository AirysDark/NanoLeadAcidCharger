#include "ChargerController.h"
#include "PinsAndConfig.h"
#include "Debug.h"

namespace {
unsigned long statusLedIntervalMs(float batteryVoltage) {
  if (batteryVoltage <= CHARGE_LED_SLOW_VOLTAGE) {
    return CHARGE_LED_SLOW_INTERVAL_MS;
  }

  if (batteryVoltage >= CHARGE_LED_FAST_VOLTAGE) {
    return CHARGE_LED_FAST_INTERVAL_MS;
  }

  const float voltageSpan = CHARGE_LED_FAST_VOLTAGE - CHARGE_LED_SLOW_VOLTAGE;
  const float position = (batteryVoltage - CHARGE_LED_SLOW_VOLTAGE) / voltageSpan;
  const float intervalSpan =
      static_cast<float>(CHARGE_LED_SLOW_INTERVAL_MS - CHARGE_LED_FAST_INTERVAL_MS);

  return static_cast<unsigned long>(
      static_cast<float>(CHARGE_LED_SLOW_INTERVAL_MS) - (position * intervalSpan));
}

void updateChargeProgressLed(bool charging, float batteryVoltage, unsigned long nowMs) {
  // Dedicated D6 charge-progress LED. Keep it OFF whenever charging is not active.
  if (!charging) {
    analogWrite(PIN_CHARGE_PROGRESS_LED, 0);
    return;
  }

  // While charging, smoothly fade from fully OFF to full brightness and back.
  // The pulse gets faster as battery voltage rises.
  const unsigned long cycleMs = statusLedIntervalMs(batteryVoltage);
  const unsigned long phaseMs = nowMs % cycleMs;
  const unsigned long halfCycleMs = cycleMs / 2UL;

  uint8_t brightness;
  if (phaseMs < halfCycleMs) {
    const unsigned long span =
        static_cast<unsigned long>(CHARGE_LED_MAX_BRIGHTNESS - CHARGE_LED_MIN_BRIGHTNESS);
    brightness = static_cast<uint8_t>(
        CHARGE_LED_MIN_BRIGHTNESS +
        ((span * phaseMs) / (halfCycleMs > 0 ? halfCycleMs : 1UL)));
  } else {
    const unsigned long downPhase = phaseMs - halfCycleMs;
    const unsigned long downDuration = cycleMs - halfCycleMs;
    const unsigned long span =
        static_cast<unsigned long>(CHARGE_LED_MAX_BRIGHTNESS - CHARGE_LED_MIN_BRIGHTNESS);
    brightness = static_cast<uint8_t>(
        CHARGE_LED_MAX_BRIGHTNESS -
        ((span * downPhase) / (downDuration > 0 ? downDuration : 1UL)));
  }

  analogWrite(PIN_CHARGE_PROGRESS_LED, brightness);
}
}  // namespace

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

  pinMode(PIN_POWER_LED, OUTPUT);
  digitalWrite(PIN_POWER_LED, HIGH);

  pinMode(PIN_CHARGE_PROGRESS_LED, OUTPUT);
  analogWrite(PIN_CHARGE_PROGRESS_LED, 0);

  analogReference(DEFAULT);

  Debug::begin();
  if (ENABLE_EXTERNAL_TEMP_SENSOR) {
    _temperature.begin();
  }
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

  if (ENABLE_EXTERNAL_TEMP_SENSOR) {
    _temperature.update(nowMs);
  }
  if (ENABLE_INTERNAL_TEMP_MONITOR) {
    _internalTemperature.update(nowMs);
  }

  if ((nowMs - _lastControlMs) >= CONTROL_INTERVAL_MS) {
    _lastControlMs = nowMs;
    _batteryVoltage = readBatteryVoltage();
    runControl(nowMs);
  }

  // D9 is a dedicated power indicator and remains solid while the Nano is powered.
  digitalWrite(PIN_POWER_LED, HIGH);
  updateChargeProgressLed(_gateway.enabled(), _batteryVoltage, nowMs);

  if (ENABLE_DEBUG && (nowMs - _lastDebugMs) >= DEBUG_INTERVAL_MS) {
    _lastDebugMs = nowMs;
    const unsigned long offForMs = _gateway.enabled() ? 0UL : (nowMs - _lastTurnedOffMs);
    Debug::printStatus(_batteryVoltage,
                       ENABLE_EXTERNAL_TEMP_SENSOR && _temperature.valid(),
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

  if (ENABLE_EXTERNAL_TEMP_SENSOR &&
      REQUIRE_TEMP_SENSOR &&
      !_temperature.valid()) {
    setCharger(false, ChargerState::SENSOR_FAULT, nowMs);
    return;
  }

  if (ENABLE_INTERNAL_TEMP_MONITOR &&
      REQUIRE_INTERNAL_TEMP_MONITOR &&
      !_internalTemperature.valid()) {
    setCharger(false, ChargerState::INTERNAL_TEMP_FAULT, nowMs);
    return;
  }

  if (ENABLE_EXTERNAL_TEMP_SENSOR && _temperature.valid()) {
    if (_temperature.celsius() >= TEMP_CUTOFF_C) {
      _temperatureLockout = true;
    } else if (_temperature.celsius() <= TEMP_RESTART_C) {
      _temperatureLockout = false;
    }
  }

  if (ENABLE_EXTERNAL_TEMP_SENSOR && _temperatureLockout) {
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
  return ENABLE_EXTERNAL_TEMP_SENSOR && _temperature.valid();
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
