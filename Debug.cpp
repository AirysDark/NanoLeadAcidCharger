#include "Debug.h"
#include "PinsAndConfig.h"
#include "ChargerController.h"

namespace Debug {

void begin() {
  if (!ENABLE_DEBUG) {
    return;
  }

  Serial.begin(DEBUG_BAUD);

  if (DEBUG_WAIT_FOR_SERIAL_MS > 0UL) {
    const unsigned long startMs = millis();
    while (!Serial && (millis() - startMs) < DEBUG_WAIT_FOR_SERIAL_MS) {
      // Classic Nano does not need this, but it is harmless and useful if
      // the project is ever moved to a native-USB board.
    }
  }
}

void printStartup(bool internalTempEnabled) {
  if (!ENABLE_DEBUG) {
    return;
  }

  Serial.println();
  Serial.println(F("Nano Lead-Acid Charger Controller"));
  Serial.println(F("DEBUG: enabled"));
  Serial.println(F("Fail-safe startup: charger OFF"));
  Serial.print(F("Nano internal temperature safety: "));
  Serial.println(internalTempEnabled ? F("ENABLED") : F("DISABLED"));
}

void printStatus(float batteryVoltage,
                 bool batteryTempValid,
                 float batteryTempC,
                 bool internalTempEnabled,
                 bool internalTempValid,
                 float internalTempC,
                 uint16_t internalTempRawAdc,
                 bool chargerEnabled,
                 ChargerState state,
                 unsigned long offForMs) {
  if (!ENABLE_DEBUG) {
    return;
  }

  Serial.print(F("Battery: "));
  Serial.print(batteryVoltage, 2);
  Serial.print(F(" V | BatteryTemp: "));

  if (batteryTempValid) {
    Serial.print(batteryTempC, 1);
    Serial.print(F(" C"));
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" | NanoTemp: "));
  if (!internalTempEnabled) {
    Serial.print(F("DISABLED"));
  } else if (internalTempValid) {
    Serial.print(internalTempC, 1);
    Serial.print(F(" C"));
  } else {
    Serial.print(F("INVALID"));
  }

  if (internalTempEnabled) {
    Serial.print(F(" (raw "));
    Serial.print(internalTempRawAdc);
    Serial.print(F(")"));
  }

  Serial.print(F(" | Charger: "));
  Serial.print(chargerEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" | State: "));
  Serial.print(stateName(state));

  if (!chargerEnabled) {
    Serial.print(F(" | Off: "));
    Serial.print(offForMs / 1000UL);
    Serial.print(F("s"));
  }

  Serial.println();
}

void printChargerChange(bool enabled,
                        ChargerState state,
                        float batteryVoltage) {
  if (!ENABLE_DEBUG || !DEBUG_PRINT_STATE_CHANGES) {
    return;
  }

  Serial.print(F("[STATE] Charger "));
  Serial.print(enabled ? F("ON") : F("OFF"));
  Serial.print(F(" -> "));
  Serial.print(stateName(state));
  Serial.print(F(" | Battery: "));
  Serial.print(batteryVoltage, 2);
  Serial.println(F(" V"));
}

const __FlashStringHelper* stateName(ChargerState state) {
  switch (state) {
    case ChargerState::STARTUP:             return F("STARTUP");
    case ChargerState::CHARGING:            return F("CHARGING");
    case ChargerState::FULL_OFF:            return F("FULL/OFF");
    case ChargerState::BATTERY_TEMP_OFF:    return F("BATTERY TEMP CUTOFF");
    case ChargerState::INTERNAL_TEMP_OFF:   return F("CHARGER TEMP CUTOFF");
    case ChargerState::SENSOR_FAULT:        return F("BATTERY TEMP SENSOR FAULT");
    case ChargerState::INTERNAL_TEMP_FAULT: return F("NANO TEMP SENSOR FAULT");
    case ChargerState::VOLTAGE_FAULT:       return F("VOLTAGE FAULT");
    case ChargerState::OVERVOLTAGE_FAULT:   return F("OVERVOLTAGE");
    default:                                return F("UNKNOWN");
  }
}

}  // namespace Debug
