#pragma once
#include <Arduino.h>

// ============================================================
// ONLY EDIT THIS FILE TO CHANGE PINS / LIMITS / CALIBRATION
// ============================================================

// ------------------------- Pins ------------------------------
constexpr uint8_t PIN_CHARGE_MOSFET = 5;
constexpr uint8_t PIN_BATTERY_VOLTAGE = A0;
constexpr uint8_t PIN_TEMP_SENSOR = 2;
constexpr uint8_t PIN_STATUS_LED = 9;

// -------------------- MOSFET behaviour -----------------------
constexpr bool CHARGE_CONTROL_ACTIVE_HIGH = true;

// ---------------- Battery voltage divider --------------------
// Wiring: Battery+ -> 33k -> A0 -> 10k -> Battery-
constexpr float BATTERY_DIVIDER_R_TOP_OHMS = 33000.0f;
constexpr float BATTERY_DIVIDER_R_BOTTOM_OHMS = 10000.0f;

constexpr float ADC_REFERENCE_VOLTS = 5.00f;
constexpr uint16_t ADC_MAX_COUNTS = 1023;

// Voltage calibration result. The ESP8266 VOLTAGE CAL test will give you
// replacement values for BOTH of these lines after three multimeter readings.
// Applied as: calibrated = (nominal divider volts * CALIBRATION) + OFFSET
constexpr float BATTERY_VOLTAGE_CALIBRATION = 1.000000f;
constexpr float BATTERY_VOLTAGE_OFFSET_VOLTS = 0.0000f;

constexpr float VOLT_CAL_MIN_STEP_VOLTS = 0.05f;
constexpr float VOLT_CAL_MIN_TOTAL_SPAN_VOLTS = 0.10f;
constexpr uint8_t VOLT_CAL_REQUIRED_SAMPLES = 3;

constexpr uint8_t ADC_SAMPLES = 32;

// -------------------- Charging thresholds --------------------
constexpr float CHARGE_CUTOFF_VOLTS = 14.40f;
constexpr float CHARGE_RESTART_VOLTS = 13.20f;
constexpr float HARD_OVERVOLTAGE_VOLTS = 14.60f;
constexpr float MIN_VALID_BATTERY_VOLTS = 8.0f;
constexpr float MAX_VALID_BATTERY_VOLTS = 16.0f;
constexpr unsigned long MIN_OFF_TIME_MS = 60000UL;

// ---------------------- Status LED ---------------------------
// D9 LED behaviour:
//   Nano powered, not charging = solid ON
//   Charging = short flashes
//   Lower battery voltage = longer gap between flashes
//   Near full = faster flashes
constexpr float STATUS_LED_SLOW_VOLTAGE = 12.00f;
constexpr float STATUS_LED_FAST_VOLTAGE = CHARGE_CUTOFF_VOLTS;
constexpr unsigned long STATUS_LED_SLOW_INTERVAL_MS = 2000UL;
constexpr unsigned long STATUS_LED_FAST_INTERVAL_MS = 250UL;
constexpr unsigned long STATUS_LED_FLASH_ON_MS = 100UL;

// ---------------- External battery temperature ---------------
constexpr bool REQUIRE_TEMP_SENSOR = true;
constexpr float TEMP_CUTOFF_C = 45.0f;
constexpr float TEMP_RESTART_C = 40.0f;
constexpr float TEMP_MIN_VALID_C = -20.0f;
constexpr float TEMP_MAX_VALID_C = 85.0f;

// ------------- Nano internal / charger temperature -----------
constexpr bool ENABLE_INTERNAL_TEMP_MONITOR = true;
constexpr bool REQUIRE_INTERNAL_TEMP_MONITOR = true;
constexpr float INTERNAL_TEMP_CUTOFF_C = 60.0f;
constexpr float INTERNAL_TEMP_RESTART_C = 50.0f;
constexpr float INTERNAL_TEMP_MIN_VALID_C = -20.0f;
constexpr float INTERNAL_TEMP_MAX_VALID_C = 120.0f;

// ATmega328P internal temperature calibration.
// TEMP SYNC will calculate replacement values for these four lines.
constexpr bool INTERNAL_TEMP_USE_RAW_CALIBRATION = true;
constexpr float INTERNAL_TEMP_CAL_RAW = 319.0f;
constexpr float INTERNAL_TEMP_CAL_C = 25.0f;
constexpr float INTERNAL_TEMP_COUNTS_PER_C = 0.93f;
constexpr float INTERNAL_TEMP_CALIBRATION_OFFSET_C = 0.0f;

constexpr uint16_t INTERNAL_TEMP_RAW_MIN_VALID = 80;
constexpr uint16_t INTERNAL_TEMP_RAW_MAX_VALID = 500;
constexpr uint8_t INTERNAL_TEMP_ADC_SAMPLES = 16;

// -------------------- Temperature sync test -------------------
// Three-point calibration using the external temperature sensor as the
// reference while it is temporarily placed beside the Nano inside the case.
//
// Point 1: 60 samples with charging held OFF.
// Point 2: return to AUTO, wait for real charging AND a temperature rise,
//          then collect another 60 samples.
// Point 3: wait for another temperature rise while charging, then collect
//          another 60 samples.
//
// A line is fitted through raw internal ADC count vs external reference temp.
// The web page then gives exact replacement calibration lines for reflashing.
constexpr unsigned long TEMP_SYNC_SAMPLE_INTERVAL_MS = 1000UL;
constexpr uint16_t TEMP_SYNC_SAMPLES_PER_POINT = 60;
constexpr float TEMP_SYNC_MIN_RISE_C = 2.0f;
constexpr float TEMP_SYNC_MIN_TOTAL_SPAN_C = 4.0f;

// ------------------------- Timing -----------------------------
constexpr unsigned long CONTROL_INTERVAL_MS = 250UL;
constexpr unsigned long TEMP_INTERVAL_MS = 1000UL;
constexpr unsigned long INTERNAL_TEMP_INTERVAL_MS = 1000UL;

// -------------------- ESP8266 command link --------------------
constexpr bool ENABLE_ESP_COMMANDS = true;
constexpr uint8_t PIN_ESP_RX = 8;
constexpr uint8_t PIN_ESP_TX = 7;
constexpr unsigned long ESP_COMMAND_BAUD = 9600UL;
constexpr uint8_t COMMAND_BUFFER_SIZE = 48;

// -------------------------- Debug -----------------------------
constexpr bool ENABLE_DEBUG = true;
constexpr unsigned long DEBUG_BAUD = 115200UL;
constexpr unsigned long DEBUG_INTERVAL_MS = 1000UL;
constexpr bool DEBUG_PRINT_STATE_CHANGES = true;
constexpr unsigned long DEBUG_WAIT_FOR_SERIAL_MS = 0UL;
