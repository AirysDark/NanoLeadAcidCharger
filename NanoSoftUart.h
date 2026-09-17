#pragma once
#include <Arduino.h>
#include <Stream.h>

// Minimal software UART for the ATmega328P Nano command link.
//
// Why this exists:
// ArduinoDroid can install EspSoftwareSerial alongside the AVR core. Both
// libraries provide a file named SoftwareSerial.h, and ArduinoDroid may pick
// the ESP8266 header while compiling the Nano AVR SoftwareSerial.cpp. That
// causes the Nano build to fail before our sketch is compiled.
//
// This local class removes that dependency entirely. It is intentionally
// small and only provides the Stream/Print operations used by Command.cpp.
// RX uses a pin-change interrupt; TX is timing-based. The default 9600 baud
// command link on Nano D8/D9 is well within its intended use.
class NanoSoftUart : public Stream {
public:
  NanoSoftUart(uint8_t receivePin, uint8_t transmitPin);

  void begin(unsigned long baud);
  void end();

  int available() override;
  int read() override;
  int peek() override;
  void flush() override;
  size_t write(uint8_t byte) override;
  using Print::write;

  static void handleInterrupt();

private:
  static constexpr uint8_t RX_BUFFER_SIZE = 64;
  static NanoSoftUart* _active;

  const uint8_t _receivePin;
  const uint8_t _transmitPin;

  volatile uint8_t _rxBuffer[RX_BUFFER_SIZE];
  volatile uint8_t _rxHead;
  volatile uint8_t _rxTail;

  volatile uint8_t* _rxInputRegister;
  uint8_t _rxBitMask;
  volatile uint8_t* _txOutputRegister;
  uint8_t _txBitMask;

  volatile uint8_t* _pcmsk;
  uint8_t _pcmskBit;
  volatile uint8_t* _pcicr;
  uint8_t _pcicrBit;

  uint16_t _bitTimeUs;
  bool _started;

  bool rxHigh() const;
  void setTx(bool high);
  void receiveByteFromInterrupt();
};
