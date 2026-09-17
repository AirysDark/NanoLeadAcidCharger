#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PinsAndConfig.h"

class ChargerController;

// UART command link used by the ESP8266 web/serial bridge.
// Commands are ASCII text terminated with \n.
//
// Safety rule: there is deliberately NO remote FORCE-ON command.
// STOP can inhibit charging immediately; AUTO returns control to the normal
// voltage/temperature safety logic in ChargerController.
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

  ChargerController& _charger;
  SoftwareSerial _serial;

  char _buffer[COMMAND_BUFFER_SIZE];
  uint8_t _length;
  bool _discardUntilNewline;

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

  void sendStatus();
  void sendBattery();
  void sendTemperature();
  void sendState();
  void sendTempSyncStatus();
  void sendTempSyncFields();
  void sendHelp();
  void sendError(const __FlashStringHelper* message);
};
