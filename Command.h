#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PinsAndConfig.h"

class ChargerController;

// UART command link used by the ESP8266 web/serial bridge.
//
// Commands are ASCII text terminated with \n. The ESP8266 polls the Nano
// and the Nano returns one-line machine-friendly responses.
//
// The command interface deliberately does NOT provide a FORCE-ON command.
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

  void handleChar(char c);
  void processLine();
  void normalizeLine();

  void sendStatus();
  void sendBattery();
  void sendTemperature();
  void sendState();
  void sendHelp();
  void sendError(const __FlashStringHelper* message);
};
