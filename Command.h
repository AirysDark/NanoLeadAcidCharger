#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PinsAndConfig.h"

class ChargerController;

// UART command link used by the ESP8266 web/serial bridge.
//
// Commands are ASCII text terminated with \n. The ESP8266 polls STATUS
// and the Nano returns one-line machine-friendly responses.
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
  ChargerController& _charger;
  SoftwareSerial _serial;

  char _buffer[COMMAND_BUFFER_SIZE];
  uint8_t _length;
  bool _discardUntilNewline;

  // Temperature-sync test state. During a sync the external temperature
  // sensor is used as the reference next to the Nano inside the charger case.
  // The charger itself continues to run under its normal safety logic.
  bool _tempSyncActive;
  unsigned long _tempSyncStartMs;
  unsigned long _tempSyncLastSampleMs;
  uint32_t _tempSyncSamples;
  float _tempSyncDeltaSum;
  float _tempSyncCurrentDelta;
  float _tempSyncAverageDelta;

  void handleChar(char c);
  void processLine();
  void normalizeLine();

  void updateTempSync(unsigned long nowMs);
  void startTempSync();
  void stopTempSync();
  bool tempSyncReady() const;
  float tempSyncRecommendedOffset() const;

  void sendStatus();
  void sendBattery();
  void sendTemperature();
  void sendState();
  void sendTempSyncStatus();
  void sendTempSyncFields();
  void sendHelp();
  void sendError(const __FlashStringHelper* message);
};
