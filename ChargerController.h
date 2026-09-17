#pragma once
#include <Arduino.h>
#include "TemperatureSensor.h"
#include "InternalTemperature.h"
#include "gateway.h"

enum class ChargerState : uint8_t {
  STARTUP,
  CHARGING,
  FULL_OFF,
  BATTERY_TEMP_OFF,
  INTERNAL_TEMP_OFF,
  SENSOR_FAULT,
  INTERNAL_TEMP_FAULT,
  VOLTAGE_FAULT,
  OVERVOLTAGE_FAULT,
  REMOTE_OFF
};

class ChargerController {
public:
  ChargerController();

  void begin();
  void update();

  float batteryVoltage() const;
  float batteryTemperatureC() const;
  float internalTemperatureC() const;
  bool temperatureValid() const;
  bool internalTemperatureValid() const;
  bool chargerEnabled() const;
  ChargerState state() const;

  // Safe remote control used by Command.cpp.
  // true  = force charging OFF immediately
  // false = return to the normal automatic safety logic
  // There is intentionally no remote force-ON bypass.
  void setRemoteInhibit(bool inhibit);
  bool remoteInhibited() const;

private:
  TemperatureSensor _temperature;
  InternalTemperature _internalTemperature;
  Gateway _gateway;

  ChargerState _state;
  bool _temperatureLockout;
  bool _internalTemperatureLockout;
  bool _remoteInhibit;

  float _batteryVoltage;
  unsigned long _lastControlMs;
  unsigned long _lastDebugMs;
  unsigned long _lastTurnedOffMs;

  float readBatteryVoltage();
  void runControl(unsigned long nowMs);
  void setCharger(bool enabled, ChargerState state, unsigned long nowMs);
};
