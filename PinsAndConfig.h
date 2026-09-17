#pragma once
#include <Arduino.h>

// ============================================================
// ONLY EDIT THIS FILE TO CHANGE PINS / LIMITS / CALIBRATION
// ============================================================

// ------------------------- Pins ------------------------------
// Charger cutoff MOSFET gate pin. The gateway module owns all direct
// MOSFET/GPIO switching; change the physical pin here only.
constexpr uint8_t PIN_CHARGE_MOSFET = 5;

// Battery voltage divider midpoint: Battery+ -> R_TOP -> A0 -> R_BOTTOM -> Battery-
constexpr uint8_t PIN_BATTERY_VOLTAGE = A0;

// External DS18B20 / KY-001 style battery-temperature sensor signal pin.
constexpr uint8_t PIN_TEMP_SENSOR = 2;

// Built-in LED is used as a simple status indicator.
constexpr uint8_t PIN_STATUS_LED = LED_BUILTIN;

// -------------------- MOSFET behaviour -----------------------
// Used by gateway.h/.cpp.
// true:  gate HIGH = barrel charger ON
// false: gate LOW  = barrel charger ON (only for inverted driver hardware)
constexpr bool CHARGE_CONTROL_ACTIVE_HIGH = true;

// ---------------- Battery voltage divider --------------------
// Default wiring: Battery+ -> 33k -> A0 -> 10k -> Battery-
constexpr float BATTERY_DIVIDER_R_TOP_OHMS = 33000.0f;
constexpr float BATTERY_DIVIDER_R_BOTTOM_OHMS = 10000.0f;

// Nano ADC reference. Measure the Nano 5V rail with your multimeter and
// put the real value here for better voltage accuracy (example 4.93).
constexpr float ADC_REFERENCE_VOLTS = 5.00f;
constexpr uint16_t ADC_MAX_COUNTS = 1023;

// Fine calibration after comparing Serial voltage with your multimeter.
// Example: meter = 13.20V, Serial = 13.00V -> 13.20/13.00 = 1.0154
constexpr float BATTERY_VOLTAGE_CALIBRATION = 1.0000f;

// ADC averaging. Higher = steadier reading, slightly slower.
constexpr uint8_t ADC_SAMPLES = 32;

// -------------------- Charging thresholds --------------------
// Initial/normal charge cutoff.
constexpr float CHARGE_CUTOFF_VOLTS = 14.40f;

// After cutoff, charging will not restart until battery falls below this.
// This creates hysteresis and stops rapid ON/OFF cycling.
constexpr float CHARGE_RESTART_VOLTS = 13.20f;

// Absolute emergency high-voltage cutoff.
constexpr float HARD_OVERVOLTAGE_VOLTS = 14.60f;

// Battery voltage must look sane before charging is allowed.
constexpr float MIN_VALID_BATTERY_VOLTS = 8.0f;
constexpr float MAX_VALID_BATTERY_VOLTS = 16.0f;

// Minimum time charger must remain OFF after a normal voltage cutoff.
constexpr unsigned long MIN_OFF_TIME_MS = 60000UL; // 60 seconds

// ---------------- External battery temperature ---------------
// The pictured 3-wire module looks like a DS18B20/KY-001 style sensor.
// Verify its S / + / - markings before powering it.
constexpr bool REQUIRE_TEMP_SENSOR = true;

// Emergency battery temperature cutoff / restart hysteresis.
constexpr float TEMP_CUTOFF_C = 45.0f;
constexpr float TEMP_RESTART_C = 40.0f;

// DS18B20 normal sanity range for this project.
constexpr float TEMP_MIN_VALID_C = -20.0f;
constexpr float TEMP_MAX_VALID_C = 85.0f;

// ------------- Nano internal / charger temperature -----------
// The ATmega328P includes an internal temperature sensor. Because the Nano
// will live inside the charger enclosure, we use it as a SECONDARY thermal
// safety channel for enclosure/electronics heat.
constexpr bool ENABLE_INTERNAL_TEMP_MONITOR = true;
constexpr bool REQUIRE_INTERNAL_TEMP_MONITOR = true;

// Conservative enclosure/controller thermal cutoff and hysteresis.
constexpr float INTERNAL_TEMP_CUTOFF_C = 60.0f;
constexpr float INTERNAL_TEMP_RESTART_C = 50.0f;

// Sanity range. Values outside this are considered invalid.
constexpr float INTERNAL_TEMP_MIN_VALID_C = -20.0f;
constexpr float INTERNAL_TEMP_MAX_VALID_C = 120.0f;

// ATmega328P internal temperature sensor calibration.
// IMPORTANT: clone Nano boards can have a large sensor offset, so this build
// uses a RAW-ADC calibration point instead of assuming the datasheet typical
// 314 mV @ 25 C. Your board is currently reporting about raw 319.
//
// CALIBRATION:
// 1) Let the Nano sit powered for ~10 minutes with the charger MOSFET OFF.
// 2) Measure the temperature near the ATmega328P with a known thermometer.
// 3) Put the debug raw value in INTERNAL_TEMP_CAL_RAW and the measured
//    temperature in INTERNAL_TEMP_CAL_C.
// 4) Warm the ATmega328P gently with a finger; the raw value should rise.
//
// The sensor is only a SECONDARY enclosure-overheat safety channel.
constexpr bool INTERNAL_TEMP_USE_RAW_CALIBRATION = true;
constexpr float INTERNAL_TEMP_CAL_RAW = 319.0f;  // your Nano raw ADC count at the starting calibration point
constexpr float INTERNAL_TEMP_CAL_C = 25.0f;     // starting estimate; replace with measured charger/Nano temperature
constexpr float INTERNAL_TEMP_COUNTS_PER_C = 0.93f; // ~1 mV/C with 1.1 V ref ~= 0.93 count/C
constexpr float INTERNAL_TEMP_CALIBRATION_OFFSET_C = 0.0f;

// Raw sanity window. Raw values outside this are treated as a read/config fault.
constexpr uint16_t INTERNAL_TEMP_RAW_MIN_VALID = 80;
constexpr uint16_t INTERNAL_TEMP_RAW_MAX_VALID = 500;

// ADC averaging.
constexpr uint8_t INTERNAL_TEMP_ADC_SAMPLES = 16;

// -------------------- Temperature sync test -------------------
// TEMP SYNC is started from the ESP8266/web monitor. Temporarily place the
// external temperature sensor next to the Nano inside the charger enclosure.
// The Nano compares the external sensor with its internal sensor while normal
// charger voltage/temperature safety continues to operate.
constexpr unsigned long TEMP_SYNC_SAMPLE_INTERVAL_MS = 1000UL;

// The web page marks the calibration result READY after this many valid
// paired samples. At 1 sample/sec, 60 samples = about one minute.
constexpr uint16_t TEMP_SYNC_MIN_SAMPLES = 60;

// ------------------------- Timing -----------------------------
constexpr unsigned long CONTROL_INTERVAL_MS = 250UL;
constexpr unsigned long TEMP_INTERVAL_MS = 1000UL;
constexpr unsigned long INTERNAL_TEMP_INTERVAL_MS = 1000UL;

// -------------------- ESP8266 command link --------------------
// Dedicated SoftwareSerial link so the Nano USB Serial Monitor remains free.
// Nano RX receives ESP8266 TX.
// Nano TX sends to ESP8266 RX through the 5V -> 3.3V divider.
constexpr bool ENABLE_ESP_COMMANDS = true;
constexpr uint8_t PIN_ESP_RX = 8;
constexpr uint8_t PIN_ESP_TX = 9;

// SoftwareSerial is deliberately kept conservative for reliability.
constexpr unsigned long ESP_COMMAND_BAUD = 9600UL;

// Maximum incoming command length including room for the null terminator.
constexpr uint8_t COMMAND_BUFFER_SIZE = 48;

// -------------------------- Debug -----------------------------
// Master switch for all Serial Monitor debugging.
// true  = Serial Monitor debugging enabled
// false = no debug Serial output
constexpr bool ENABLE_DEBUG = true;

// Serial Monitor speed when debugging is enabled.
constexpr unsigned long DEBUG_BAUD = 115200UL;

// How often a full status line is printed.
constexpr unsigned long DEBUG_INTERVAL_MS = 1000UL;

// Also print an immediate line whenever the charger output or control state changes.
constexpr bool DEBUG_PRINT_STATE_CHANGES = true;

// Classic Nano normally needs no wait. Leave at 0 for immediate startup.
constexpr unsigned long DEBUG_WAIT_FOR_SERIAL_MS = 0UL;
