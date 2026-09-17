#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class TemperatureSensor {
public:
  explicit TemperatureSensor(uint8_t pin);

  void begin();
  void update(unsigned long nowMs);

  bool valid() const;
  float celsius() const;
  unsigned long ageMs(unsigned long nowMs) const;

private:
  OneWire _oneWire;
  DallasTemperature _sensors;

  float _temperatureC;
  bool _valid;
  unsigned long _lastReadMs;
  unsigned long _lastInitAttemptMs;

  void initialiseBus();
  void readSensor();
};
