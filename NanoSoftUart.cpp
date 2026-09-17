#include "NanoSoftUart.h"
#include <avr/interrupt.h>

NanoSoftUart* NanoSoftUart::_active = nullptr;

NanoSoftUart::NanoSoftUart(uint8_t receivePin, uint8_t transmitPin)
  : _receivePin(receivePin),
    _transmitPin(transmitPin),
    _rxHead(0),
    _rxTail(0),
    _rxInputRegister(nullptr),
    _rxBitMask(0),
    _txOutputRegister(nullptr),
    _txBitMask(0),
    _pcmsk(nullptr),
    _pcmskBit(0),
    _pcicr(nullptr),
    _pcicrBit(0),
    _bitTimeUs(0),
    _started(false) {
}

void NanoSoftUart::begin(unsigned long baud) {
  if (baud == 0) baud = 9600UL;

  _bitTimeUs = static_cast<uint16_t>(1000000UL / baud);
  if (_bitTimeUs == 0) _bitTimeUs = 1;

  pinMode(_receivePin, INPUT_PULLUP);
  pinMode(_transmitPin, OUTPUT);
  digitalWrite(_transmitPin, HIGH);  // UART idle state

  _rxInputRegister = portInputRegister(digitalPinToPort(_receivePin));
  _rxBitMask = digitalPinToBitMask(_receivePin);
  _txOutputRegister = portOutputRegister(digitalPinToPort(_transmitPin));
  _txBitMask = digitalPinToBitMask(_transmitPin);

  _pcmsk = digitalPinToPCMSK(_receivePin);
  _pcmskBit = digitalPinToPCMSKbit(_receivePin);
  _pcicr = digitalPinToPCICR(_receivePin);
  _pcicrBit = digitalPinToPCICRbit(_receivePin);

  _rxHead = 0;
  _rxTail = 0;
  _active = this;

  if (_pcmsk != nullptr && _pcicr != nullptr) {
    const uint8_t oldSreg = SREG;
    cli();

    // Clear any stale pin-change flag before enabling reception.
    PCIFR = _BV(_pcicrBit);
    *_pcicr |= _BV(_pcicrBit);
    *_pcmsk |= _BV(_pcmskBit);

    SREG = oldSreg;
    _started = true;
  }
}

void NanoSoftUart::end() {
  if (_pcmsk != nullptr) {
    const uint8_t oldSreg = SREG;
    cli();
    *_pcmsk &= static_cast<uint8_t>(~_BV(_pcmskBit));
    SREG = oldSreg;
  }

  if (_active == this) _active = nullptr;
  _started = false;
}

int NanoSoftUart::available() {
  const uint8_t head = _rxHead;
  const uint8_t tail = _rxTail;
  if (tail >= head) return tail - head;
  return RX_BUFFER_SIZE - head + tail;
}

int NanoSoftUart::read() {
  if (_rxHead == _rxTail) return -1;

  const uint8_t value = _rxBuffer[_rxHead];
  _rxHead = static_cast<uint8_t>((_rxHead + 1U) % RX_BUFFER_SIZE);
  return value;
}

int NanoSoftUart::peek() {
  if (_rxHead == _rxTail) return -1;
  return _rxBuffer[_rxHead];
}

void NanoSoftUart::flush() {
  // TX is synchronous: write() does not return until the byte is sent.
}

size_t NanoSoftUart::write(uint8_t byte) {
  if (!_started || _txOutputRegister == nullptr) return 0;

  const uint8_t oldSreg = SREG;
  cli();

  // Start bit.
  setTx(false);
  delayMicroseconds(_bitTimeUs);

  // Eight data bits, LSB first.
  for (uint8_t i = 0; i < 8; ++i) {
    setTx((byte & 0x01U) != 0U);
    delayMicroseconds(_bitTimeUs);
    byte >>= 1;
  }

  // Stop bit / idle.
  setTx(true);
  delayMicroseconds(_bitTimeUs);

  SREG = oldSreg;
  return 1;
}

bool NanoSoftUart::rxHigh() const {
  return (_rxInputRegister != nullptr) && ((*_rxInputRegister & _rxBitMask) != 0U);
}

void NanoSoftUart::setTx(bool high) {
  if (high) *_txOutputRegister |= _txBitMask;
  else *_txOutputRegister &= static_cast<uint8_t>(~_txBitMask);
}

void NanoSoftUart::handleInterrupt() {
  if (_active != nullptr) _active->receiveByteFromInterrupt();
}

void NanoSoftUart::receiveByteFromInterrupt() {
  if (!_started || _pcmsk == nullptr) return;

  // Pin-change interrupts fire on both edges. A UART byte starts on the
  // falling edge, so ignore an interrupt when the RX line is already HIGH.
  if (rxHigh()) return;

  // Ignore further transitions on RX while this byte is sampled.
  *_pcmsk &= static_cast<uint8_t>(~_BV(_pcmskBit));

  // Move to the centre of data bit 0: one start bit + half a bit.
  delayMicroseconds(static_cast<uint16_t>(_bitTimeUs + (_bitTimeUs / 2U)));

  uint8_t value = 0;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (rxHigh()) value |= static_cast<uint8_t>(1U << bit);
    delayMicroseconds(_bitTimeUs);
  }

  // Store unless the ring buffer is full.
  const uint8_t nextTail = static_cast<uint8_t>((_rxTail + 1U) % RX_BUFFER_SIZE);
  if (nextTail != _rxHead) {
    _rxBuffer[_rxTail] = value;
    _rxTail = nextTail;
  }

  // Transitions during sampling may have set the group's pending flag.
  // Clear it before re-enabling the receive pin's pin-change interrupt.
  PCIFR = _BV(_pcicrBit);
  *_pcmsk |= _BV(_pcmskBit);
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect) {
  NanoSoftUart::handleInterrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect) {
  NanoSoftUart::handleInterrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect) {
  NanoSoftUart::handleInterrupt();
}
#endif
