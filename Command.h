#pragma once
#include <Arduino.h>
#include "NanoSoftUart.h"
#include "PinsAndConfig.h"

class ChargerController;

class Command {
public:
  explicit Command(ChargerController& charger);
  void begin();
  void update();

private:
  enum class TempSyncPhase : uint8_t {
    OFF,
    POINT1,
    WAIT_POINT2,
    POINT2,
    WAIT_POINT3,
    POINT3,
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
  NanoSoftUart _serial;
  char _buffer[COMMAND_BUFFER_SIZE];
  uint8_t _length;
  bool _discardUntilNewline;

  TempSyncPhase _tempSyncPhase;
  bool _tempSyncWasRemoteInhibited;
  unsigned long _tempSyncLastSampleMs;
  uint16_t _tempSamples[3];
  float _tempExtSum[3];
  float _tempRawSum[3];
  float _tempExtAvg[3];
  float _tempRawAvg[3];
  float _tempSyncCurrentDelta;
  float _tempCalRaw;
  float _tempCalC;
  float _tempCountsPerC;

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
  void addTempSample(uint8_t pointIndex);
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
