#pragma once
#include <Arduino.h>

// Reads the ATmega328P's internal temperature sensor.
// This is intentionally used only as a secondary charger/enclosure
// over-temperature safety channel. It is not a substitute for the
// external battery temperature sensor.
class InternalTemperature {
public:
  InternalTemperature();

  void begin();
  void update(unsigned long nowMs);

  bool valid() const;
  float celsius() const;
  uint16_t rawAdc() const;

private:
  float _temperatureC;
  uint16_t _rawAdc;
  bool _valid;
  unsigned long _lastReadMs;

  float readCelsius();
};
