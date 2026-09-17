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
  DeviceAddress _address;

  float _temperatureC;
  bool _valid;
  bool _addressValid;
  uint8_t _ds18Count;
  unsigned long _lastReadMs;
  unsigned long _lastInitAttemptMs;

  void initialiseBus();
  void readSensor();
  void printBusDiagnostics() const;
};
