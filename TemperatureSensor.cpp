#include "TemperatureSensor.h"
#include "PinsAndConfig.h"

TemperatureSensor::TemperatureSensor(uint8_t pin)
  : _oneWire(pin),
    _sensors(&_oneWire),
    _temperatureC(NAN),
    _valid(false),
    _lastReadMs(0),
    _lastInitAttemptMs(0) {
}

void TemperatureSensor::initialiseBus() {
  _lastInitAttemptMs = millis();
  _sensors.begin();

  // 10-bit is plenty for charger protection and shortens conversion time.
  _sensors.setResolution(10);
}

void TemperatureSensor::readSensor() {
  _sensors.requestTemperatures();
  const float t = _sensors.getTempCByIndex(0);

  _valid = (t != DEVICE_DISCONNECTED_C) &&
           !isnan(t) &&
           (t >= TEMP_MIN_VALID_C) &&
           (t <= TEMP_MAX_VALID_C);

  if (_valid) {
    _temperatureC = t;
  } else {
    _temperatureC = NAN;
  }
}

void TemperatureSensor::begin() {
  initialiseBus();
  readSensor();
  _lastReadMs = millis();
}

void TemperatureSensor::update(unsigned long nowMs) {
  if ((nowMs - _lastReadMs) < TEMP_INTERVAL_MS) {
    return;
  }

  _lastReadMs = nowMs;

  // DallasTemperature discovers devices during begin(). If the sensor was
  // disconnected, powered late, or missed during startup, periodically rescan
  // the OneWire bus instead of remaining INVALID until the Nano is reset.
  if (!_valid &&
      (nowMs - _lastInitAttemptMs) >= TEMP_SENSOR_RETRY_INTERVAL_MS) {
    initialiseBus();
  }

  readSensor();
}

bool TemperatureSensor::valid() const {
  return _valid;
}

float TemperatureSensor::celsius() const {
  return _temperatureC;
}

unsigned long TemperatureSensor::ageMs(unsigned long nowMs) const {
  return nowMs - _lastReadMs;
}
