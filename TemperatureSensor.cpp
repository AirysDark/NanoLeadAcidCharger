#include "TemperatureSensor.h"
#include "PinsAndConfig.h"

TemperatureSensor::TemperatureSensor(uint8_t pin)
  : _oneWire(pin),
    _sensors(&_oneWire),
    _temperatureC(NAN),
    _valid(false),
    _addressValid(false),
    _ds18Count(0),
    _lastReadMs(0),
    _lastInitAttemptMs(0) {
  for (uint8_t i = 0; i < 8; ++i) {
    _address[i] = 0;
  }
}

void TemperatureSensor::printBusDiagnostics() const {
  if (!ENABLE_DEBUG) return;

  Serial.print(F("[EXT TEMP] pin=D"));
  Serial.print(PIN_TEMP_SENSOR);
  Serial.print(F(" ds18="));
  Serial.print(_ds18Count);
  Serial.print(F(" address="));

  if (!_addressValid) {
    Serial.println(F("NONE"));
    return;
  }

  for (uint8_t i = 0; i < 8; ++i) {
    if (_address[i] < 16) Serial.print('0');
    Serial.print(_address[i], HEX);
  }
  Serial.println();
}

void TemperatureSensor::initialiseBus() {
  _lastInitAttemptMs = millis();

  // Match the current DallasTemperature reference behaviour, but use a fixed
  // conversion delay rather than repeatedly polling DQ for "conversion done".
  // That is more tolerant of KY-001 style modules whose onboard LED/resistor
  // can load the 1-Wire line.
  _sensors.begin();
  _sensors.setWaitForConversion(true);
  _sensors.setCheckForConversion(false);

  _ds18Count = _sensors.getDS18Count();
  _addressValid = (_ds18Count > 0) && _sensors.getAddress(_address, 0);

  printBusDiagnostics();
}

void TemperatureSensor::readSensor() {
  if (!_addressValid) {
    _valid = false;
    _temperatureC = NAN;
    return;
  }

  // Address the actual discovered device instead of relying on index lookup
  // for every read. This also lets the library verify the sensor directly.
  if (!_sensors.isConnected(_address)) {
    _valid = false;
    _temperatureC = NAN;

    if (ENABLE_DEBUG) {
      Serial.println(F("[EXT TEMP] DS18B20 not connected"));
    }
    return;
  }

  const bool requestOk = _sensors.requestTemperaturesByAddress(_address);
  const float t = requestOk ? _sensors.getTempC(_address) : DEVICE_DISCONNECTED_C;

  _valid = requestOk &&
           (t != DEVICE_DISCONNECTED_C) &&
           !isnan(t) &&
           (t >= TEMP_MIN_VALID_C) &&
           (t <= TEMP_MAX_VALID_C);

  if (_valid) {
    _temperatureC = t;
  } else {
    _temperatureC = NAN;

    if (ENABLE_DEBUG) {
      Serial.print(F("[EXT TEMP] read failed value="));
      Serial.println(t, 2);
    }
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
