#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PinsAndConfig.h"

class ChargerController;

// UART command link used by the ESP8266 web/serial bridge.
// Commands are ASCII text terminated with \n.
//
// Safety rule: there is deliberately NO remote FORCE-ON command.
class Command {
public:
  explicit Command(ChargerController& charger);

  void begin();
  void update();

private:
  enum class TempSyncPhase : uint8_t {
    OFF,
    BASELINE,
    WAIT_FOR_CHARGE,
    CHARGING,
    COMPLETE
  };

  enum class VoltageCalPhase : uint8_t {
    OFF,
    WAIT_INPUT_1,
    WAIT_RISE_2,
    WAIT_INPUT_2,
    WAIT_RISE_3,
    WAIT_INPUT_3,
    COMPLETE
  };

  ChargerController& _charger;
  SoftwareSerial _serial;

  char _buffer[COMMAND_BUFFER_SIZE];
  uint8_t _length;
  bool _discardUntilNewline;

  // Two-stage internal-temperature calibration.
  TempSyncPhase _tempSyncPhase;
  bool _tempSyncWasRemoteInhibited;
  unsigned long _tempSyncLastSampleMs;
  uint16_t _tempSyncBaselineSamples;
  float _tempSyncBaselineSum;
  float _tempSyncBaselineAverage;
  uint16_t _tempSyncChargeSamples;
  float _tempSyncChargeSum;
  float _tempSyncChargeAverage;
  float _tempSyncCurrentDelta;
  float _tempSyncFinalAverage;
  float _tempSyncRecommendedOffset;

  // Three-point battery voltage-divider calibration.
  // Each pair stores: Nano-reported voltage at the instant the user enters
  // the multimeter voltage, and the user's actual multimeter voltage.
  VoltageCalPhase _voltageCalPhase;
  uint8_t _voltageCalSamples;
  float _voltageCalNano[VOLT_CAL_REQUIRED_SAMPLES];
  float _voltageCalActual[VOLT_CAL_REQUIRED_SAMPLES];
  float _voltageCalRecommendedScale;
  float _voltageCalRecommendedOffset;

  void handleChar(char c);
  void processLine();
  void normalizeLine();

  void updateTempSync(unsigned long nowMs);
  void startTempSync();
  void stopTempSync(bool cancelled = true);
  void completeTempSync();
  bool tempSyncActive() const;
  bool tempSyncReady() const;
  const __FlashStringHelper* tempSyncPhaseToken() const;

  void updateVoltageCal();
  void startVoltageCal();
  void addVoltageCalSample(float actualVolts);
  void stopVoltageCal();
  void completeVoltageCal();
  bool voltageCalActive() const;
  bool voltageCalReady() const;
  const __FlashStringHelper* voltageCalPhaseToken() const;

  void sendStatus();
  void sendBattery();
  void sendTemperature();
  void sendState();
  void sendTempSyncStatus();
  void sendTempSyncFields();
  void sendVoltageCalStatus();
  void sendVoltageCalFields();
  void sendHelp();
  void sendError(const __FlashStringHelper* message);
};
