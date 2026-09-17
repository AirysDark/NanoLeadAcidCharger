#pragma once
#include <Arduino.h>

enum class ChargerState : uint8_t;

namespace Debug {
  void begin();
  void printStartup(bool internalTempEnabled);

  void printStatus(float batteryVoltage,
                   bool batteryTempValid,
                   float batteryTempC,
                   bool internalTempEnabled,
                   bool internalTempValid,
                   float internalTempC,
                   uint16_t internalTempRawAdc,
                   bool chargerEnabled,
                   ChargerState state,
                   unsigned long offForMs);

  void printChargerChange(bool enabled,
                          ChargerState state,
                          float batteryVoltage);

  const __FlashStringHelper* stateName(ChargerState state);
}
