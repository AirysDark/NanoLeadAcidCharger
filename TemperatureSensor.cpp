#include "TemperatureSensor.h"
#include "PinsAndConfig.h"

TemperatureSensor::TemperatureSensor(uint8_t pin)
  : _oneWire(pin),
    _sensors(&_oneWire),
    _temperatureC(NAN),
    _valid(false),
    _lastReadMs(0) {
}

void TemperatureSensor::begin() {
  _sensors.begin();
  // 10-bit is plenty for charger protection and shortens conversion time.
  _sensors.setResolution(10);

  _sensors.requestTemperatures();
  float t = _sensors.getTempCByIndex(0);

  _valid = (t != DEVICE_DISCONNECTED_C) &&
           !isnan(t) &&
           (t >= TEMP_MIN_VALID_C) &&
           (t <= TEMP_MAX_VALID_C);

  if (_valid) {
    _temperatureC = t;
  }
  _lastReadMs = millis();
}

void TemperatureSensor::update(unsigned long nowMs) {
  if ((nowMs - _lastReadMs) < TEMP_INTERVAL_MS) {
    return;
  }

  _lastReadMs = nowMs;
  _sensors.requestTemperatures();
  float t = _sensors.getTempCByIndex(0);

  _valid = (t != DEVICE_DISCONNECTED_C) &&
           !isnan(t) &&
           (t >= TEMP_MIN_VALID_C) &&
           (t <= TEMP_MAX_VALID_C);

  if (_valid) {
    _temperatureC = t;
  }
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
