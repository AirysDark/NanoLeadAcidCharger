#include "gateway.h"
#include "PinsAndConfig.h"

Gateway::Gateway()
  : _enabled(false) {
}

uint8_t Gateway::onLevel() const {
  return CHARGE_CONTROL_ACTIVE_HIGH ? HIGH : LOW;
}

uint8_t Gateway::offLevel() const {
  return CHARGE_CONTROL_ACTIVE_HIGH ? LOW : HIGH;
}

void Gateway::begin() {
  // Fail-safe startup: preload OFF before changing the pin to OUTPUT.
  // This helps prevent a brief unwanted MOSFET turn-on during startup.
  digitalWrite(PIN_CHARGE_MOSFET, offLevel());
  pinMode(PIN_CHARGE_MOSFET, OUTPUT);

  _enabled = false;
  writeGate(false);
}

void Gateway::setEnabled(bool enabled) {
  _enabled = enabled;
  writeGate(enabled);
}

void Gateway::reassert() {
  writeGate(_enabled);
}

bool Gateway::enabled() const {
  return _enabled;
}

void Gateway::writeGate(bool enabled) {
  digitalWrite(PIN_CHARGE_MOSFET,
               enabled ? onLevel() : offLevel());
}
