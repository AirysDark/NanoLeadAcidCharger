Nano Lead-Acid Charger Controller
=================================

FILES
-----
NanoLeadAcidCharger.ino   - main sketch; no settings here
PinsAndConfig.h           - ONLY FILE YOU SHOULD NEED TO EDIT
ChargerController.h/.cpp  - voltage / cutoff / safety logic
gateway.h/.cpp            - ALL IRFB7437 MOSFET gate/control code
TemperatureSensor.h/.cpp  - external DS18B20 battery-temperature sensor
InternalTemperature.h/.cpp- ATmega328P internal Nano temperature monitor
Debug.h/.cpp              - all Serial Monitor debugging/output

ARDUINO LIBRARIES REQUIRED
--------------------------
Install using Arduino IDE Library Manager:
  1. OneWire
  2. DallasTemperature

DEFAULT CONNECTIONS
-------------------
Nano D5 = charger cutoff control output
Nano A0 = battery voltage sense
Nano D2 = external temperature sensor signal

Battery voltage sensing:
Battery + -> 33k -> A0 -> 10k -> Battery -
A0 -> 0.1uF (104) -> Battery -

External temperature sensor (probable DS18B20/KY-001 style 3-wire module):
Signal (S) -> Nano D2
+          -> Nano 5V
-          -> Nano GND

IMPORTANT: verify the S / + / - markings on the actual module before power.
Do not trust wire colours alone.

NANO INTERNAL TEMPERATURE
-------------------------
The classic ATmega328P Nano contains an internal temperature sensor. This
version reads it automatically and uses it as a SECONDARY charger/enclosure
over-temperature safety channel because the Nano will live inside the charger.

It does NOT replace the external battery-temperature sensor. The internal
sensor measures the ATmega328P die temperature, which only approximately
tracks the temperature inside the charger case.

Default internal-temperature control:
  Charger/enclosure cutoff: 60 C
  Restart after cooling:    50 C

These values and all calibration settings are in PinsAndConfig.h.

IMPORTANT: the ATmega328P internal sensor is not precision-calibrated. Let the
Nano sit at a known stable room temperature, compare Serial "NanoTemp" against
a trustworthy thermometer, then adjust:

  INTERNAL_TEMP_CALIBRATION_OFFSET_C

Example:
  Known temperature = 24.0 C
  NanoTemp           = 29.5 C
  Set offset to      = -5.5 C

The charger uses this internal reading as an extra emergency cutoff, not as a
precision thermometer.

CUTOFF HARDWARE NOTE
--------------------
All direct IRFB7437 MOSFET switching code is contained in gateway.h/.cpp.
PinsAndConfig.h still contains the editable MOSFET pin and active-level setting.
The power-stage wiring must actually disconnect the 19 V barrel source without
providing another current path around the switch.

If you use one IRFB7437 as an N-channel low-side switch, remember that its gate
voltage is referenced to its SOURCE. Do not accidentally tie the two sides of
the switched negative path together through Nano ground, USB ground, another
power supply, or test equipment; doing so bypasses the cutoff.

A direct single-NMOS low-side barrel cutoff while the Nano remains powered from
the battery generally needs the gate drive / grounding arrangement designed
carefully. Verify the final power-stage wiring before relying on automatic
shutdown.

DEFAULT CONTROL
---------------
- Startup: charger OFF
- Charge cutoff: 14.40 V
- Hard over-voltage cutoff: 14.60 V
- Restart: below 13.20 V after at least 60 seconds OFF
- External battery-temperature cutoff: 45 C
- External battery-temperature restart: 40 C
- Nano/internal charger-temperature cutoff: 60 C
- Nano/internal charger-temperature restart: 50 C
- Invalid required temperature sensor: charger OFF
- Invalid battery voltage: charger OFF

DEBUG / SERIAL MONITOR
----------------------
All Serial Monitor output is handled by Debug.h / Debug.cpp.

Enable or disable it only in PinsAndConfig.h:

  constexpr bool ENABLE_DEBUG = true;

Set it to false for normal silent operation. The charger safety/control logic
continues to work; only Serial debugging is disabled.

Other debug settings in PinsAndConfig.h:
  DEBUG_BAUD
  DEBUG_INTERVAL_MS
  DEBUG_PRINT_STATE_CHANGES
  DEBUG_WAIT_FOR_SERIAL_MS

Default baud is 115200. Typical periodic output:

Battery: 13.27 V | BatteryTemp: 27.4 C | NanoTemp: 31.2 C | Charger: ON | State: CHARGING

State changes are also printed immediately when DEBUG_PRINT_STATE_CHANGES is true.

BATTERY VOLTAGE CALIBRATION
---------------------------
1. Power the Nano from a proper regulated supply.
2. Compare Serial Monitor battery voltage with your multimeter.
3. If they differ, edit BATTERY_VOLTAGE_CALIBRATION in PinsAndConfig.h.
   Example:
     Multimeter = 13.20 V
     Serial      = 13.00 V
     Calibration = 13.20 / 13.00 = 1.0154
4. For better accuracy, also measure the Nano 5V rail and enter the measured
   value in ADC_REFERENCE_VOLTS.

SAFETY
------
This controller is a supervisory cutoff, not a certified multi-stage charger.
Use the existing current-limited charging stage, fuse the input, calibrate the
voltage reading, test both temperature shutdowns, and verify that D5 truly
removes charging current before leaving the finished system unattended.


INTERNAL NANO TEMPERATURE DEBUG
-------------------------------
NanoTemp now also prints the raw internal ADC count, for example:
  NanoTemp: 28.4 C (raw 295)

The ATmega328P internal sensor is only a secondary enclosure/overheat sensor
and varies from chip to chip. If its reading is consistently offset, adjust
INTERNAL_TEMP_CALIBRATION_OFFSET_C in PinsAndConfig.h.

============================================================
NANO INTERNAL TEMPERATURE - RAW CALIBRATION (FIX)
============================================================
If Debug showed something like:
  NanoTemp: INVALID (raw 154)
this build no longer assumes the datasheet's absolute sensor voltage.
Clone Nano / ATmega-compatible boards can have a large offset.

Default calibration now uses:
  INTERNAL_TEMP_CAL_RAW = 154
  INTERNAL_TEMP_CAL_C   = 25.0 C

This is only a starting point. For a better calibration:
1. Leave the Nano powered with charging OFF for about 10 minutes.
2. Measure temperature near the ATmega chip with a known thermometer.
3. Read the raw number from Serial Monitor.
4. Put both values into PinsAndConfig.h.
5. Gently warm the ATmega with a finger. The raw number should increase.

If raw DOES NOT change when the ATmega warms, disable:
  REQUIRE_INTERNAL_TEMP_MONITOR = false
and verify the MCU marking because the board may use a non-ATmega328P-compatible clone.

NOTE: "BatteryTemp: INVALID" is a separate external-sensor issue. If the
external sensor is not connected yet, either wire/identify it or temporarily
set REQUIRE_TEMP_SENSOR = false while bench-testing only.


NANO INTERNAL TEMPERATURE - BOARD CALIBRATION UPDATE
----------------------------------------------------
The latest Serial Monitor reading from this Nano was:
  NanoTemp: INVALID (raw 319)

The project now uses raw 319 as the one-point starting calibration reference:
  INTERNAL_TEMP_CAL_RAW = 319
  INTERNAL_TEMP_CAL_C   = 25.0 C

This should make the internal sensor report a valid temperature instead of
INVALID. 25 C is only a starting estimate. For best accuracy, let the Nano sit
at a known stable temperature, then change INTERNAL_TEMP_CAL_C in
PinsAndConfig.h to that measured temperature. Do not change code elsewhere.

The internal ATmega sensor is used only as a secondary charger/enclosure
over-temperature safety channel. The external battery sensor remains the
primary battery-temperature safety sensor.
