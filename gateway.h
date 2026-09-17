#pragma once
#include <Arduino.h>

// Handles the IRFB7437 charger-cutoff MOSFET gate.
// All hardware pin numbers and active-high/active-low behaviour stay in
// PinsAndConfig.h so that remains the only configuration file to edit.
class Gateway {
public:
  Gateway();

  // Initializes the MOSFET output in the fail-safe OFF state.
  void begin();

  // Enables or disables the 19 V barrel charging path.
  void setEnabled(bool enabled);

  // Re-applies the current gate level without changing the logical state.
  void reassert();

  bool enabled() const;

private:
  bool _enabled;

  uint8_t onLevel() const;
  uint8_t offLevel() const;
  void writeGate(bool enabled);
};
