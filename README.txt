Nano Lead-Acid Charger Controller
=================================

PURPOSE
-------
Arduino Nano lead-acid charger supervisor with battery-voltage monitoring,
external battery temperature monitoring, Nano/internal temperature monitoring,
charger cutoff control, an ESP8266 UART command link, a D9 charge-progress LED,
and guided calibration workflows for both the voltage divider and the Nano
internal temperature sensor.

FILES
-----
NanoLeadAcidCharger.ino    - main sketch
PinsAndConfig.h            - pins, thresholds and calibration values
ChargerController.h/.cpp   - voltage, charging, LED and safety logic
Command.h/.cpp             - ESP8266 UART commands and calibration tests
NanoSoftUart.h/.cpp        - local Nano software UART for D8 RX / D7 TX
gateway.h/.cpp             - charger MOSFET/gate control
TemperatureSensor.h/.cpp   - external DS18B20 battery-temperature sensor
InternalTemperature.h/.cpp - ATmega328P internal temperature monitor
Debug.h/.cpp               - USB Serial Monitor debugging

ARDUINO LIBRARIES REQUIRED
--------------------------
Install with Arduino IDE Library Manager:
  1. OneWire
  2. DallasTemperature

The Nano firmware does NOT depend on Arduino SoftwareSerial anymore. The local
NanoSoftUart class is used for the ESP8266 command link so ArduinoDroid can
compile the Nano project even when EspSoftwareSerial is also installed.

ARDUINODROID SOFTWARESERIAL CONFLICT
------------------------------------
Older builds used <SoftwareSerial.h>. ArduinoDroid could accidentally combine:

  /files/sdk/lib/SoftwareSerial/src/SoftwareSerial.cpp

with the ESP-specific header:

  /EspSoftwareSerial/src/SoftwareSerial.h

That produced Delegate/GpioCapabilities/template errors while compiling for the
ATmega328P. The current repository avoids that collision completely by using
NanoSoftUart.h/.cpp. You do not need to remove EspSoftwareSerial just to build
this Nano project.

DEFAULT CONNECTIONS
-------------------
Nano D5 = charger cutoff control output
Nano A0 = battery voltage divider
Nano D2 = external temperature sensor signal
Nano D9 = external power / charging-progress LED
Nano D8 = RX from ESP8266
Nano D7 = TX to ESP8266

Status LED:
  Nano D9 -> resistor -> LED anode (+)
  LED cathode (-) -> GND

LED behaviour:
  Nano powered, not charging = solid ON
  Charging = short flashes
  Lower battery voltage = larger gaps between flashes
  Battery voltage closer to 14.40 V = faster flashes

Default LED timing:
  12.00 V or below = one 100 ms flash every 2000 ms
  14.40 V or above = one 100 ms flash every 250 ms
  Between those voltages = linearly increasing flash rate

Battery divider:
  Battery + -> 33k -> A0 -> 10k -> Battery -
  A0 -> 0.1uF (104) -> Battery -

ESP8266 normal charger UART:
  Nano D7 TX -> 5V-to-3.3V divider -> ESP GPIO14 / D5 RX
  Nano D8 RX <- ESP GPIO12 / D6 TX
  Nano GND <-> ESP GND
  Baud = 9600

IMPORTANT: D9 was previously the Nano UART TX pin. It is now reserved for the
status LED. Move the Nano-to-ESP TX wire from D9 to D7.

The Nano TX must be reduced to 3.3V before reaching the ESP8266 RX.

WEB FIRMWARE UPDATE
-------------------
The companion AirysDark/NanoLeadAcidChargerSM ESP8266 firmware now has a
FIRMWARE UPDATE page at:

  http://192.168.4.1/firmware

The Nano can be updated from that page with NanoLeadAcidCharger.bin.

The normal D7/D8 charger UART remains connected. The ESP8266 also needs a
separate connection to the Nano's standard Arduino bootloader pins:

  ESP GPIO5 / D1 TX -> Nano D0 / RX directly
  Nano D1 / TX -> 5V-to-3.3V divider -> ESP GPIO4 / D2 RX
  ESP GPIO13 / D7 -> 1k -> logic N-MOSFET gate
  MOSFET source -> GND
  MOSFET drain  -> Nano RESET
  MOSFET gate   -> 10k -> GND
  ESP GND <-> Nano GND

The reset MOSFET prevents the Nano's 5V RESET pull-up from being connected
directly to an ESP8266 GPIO.

When a Nano .bin is uploaded, the ESP8266 first sends STOP to the charger,
resets the Nano into its bootloader, writes the application, verifies every
flash page, and then restarts the Nano. The bootloader itself is not replaced.

The updater automatically tries both common Nano bootloader baud rates:
  57600  - classic/old Nano bootloader
  115200 - Optiboot/new Nano bootloader

Maximum web-update binary size is 30720 bytes.

A GitHub Actions workflow is included at:
  .github/workflows/build-firmware.yml

It compiles the Nano and publishes:
  NanoLeadAcidCharger.bin

inside the NanoLeadAcidCharger-firmware workflow artifact. That raw .bin is the
file intended for the ESP8266 web updater.

DEFAULT CHARGER CONTROL
-----------------------
Startup: charger OFF
Normal charge cutoff: 14.40 V
Hard over-voltage cutoff: 14.60 V
Restart: <= 13.20 V after minimum OFF time
External battery-temp cutoff: 45 C
External battery-temp restart: 40 C
Nano/internal-temp cutoff: 60 C
Nano/internal-temp restart: 50 C
Required sensor fault: charger OFF
Invalid battery voltage: charger OFF

The ESP8266 can inhibit charging with STOP and return control with AUTO.
There is deliberately no remote FORCE-ON command; Nano safety logic remains
authoritative.

ESP8266 COMMANDS
----------------
Commands are ASCII lines terminated by newline.

  PING
  STATUS
  BATTERY
  TEMP
  STATE
  TSYNC START
  TSYNC STATUS
  TSYNC STOP
  VCAL START
  VCAL SAMPLE <actual-volts>
  VCAL STATUS
  VCAL STOP
  STOP
  AUTO
  HELP

STATUS includes battery voltage, external temperature, Nano temperature,
charger ON/OFF state, charger state, operating mode, and calibration telemetry.

BATTERY VOLTAGE CALIBRATION
---------------------------
The ESP8266 web monitor runs a guided 3-point voltage-divider calibration.

1. Press START VOLTAGE CAL.
2. Measure the battery directly at its terminals with a multimeter.
3. Enter the actual voltage, for example 12.07 V.
4. The Nano stores its own divider reading at that same moment.
5. The test waits until the Nano reading rises enough, then asks for reading 2.
6. Repeat for reading 2 and reading 3.
7. The Nano performs a 3-point linear fit and returns exact replacement values.

PinsAndConfig.h contains:

  constexpr float BATTERY_VOLTAGE_CALIBRATION = 1.000000f;
  constexpr float BATTERY_VOLTAGE_OFFSET_VOLTS = 0.0000f;

The ESP monitor gives the exact replacement values after calibration. Replace
both values and reflash the Nano, then verify against the multimeter again.

The battery-voltage calculation is:

  calibrated = (nominalDividerVoltage * BATTERY_VOLTAGE_CALIBRATION)
               + BATTERY_VOLTAGE_OFFSET_VOLTS

Voltage calibration never forces the charger ON or bypasses safety logic.

INTERNAL TEMPERATURE CALIBRATION
--------------------------------
The external temperature sensor is temporarily placed beside the Nano inside
the charger enclosure and used as the reference thermometer.

START TEMP SYNC performs a 3-point automatic calibration:

Point 1:
  - charger is held OFF
  - 60 paired samples are averaged
  - external temperature and internal raw ADC are stored

Point 2:
  - Nano returns to AUTO
  - test waits for real CHARGING and about a 2 C temperature rise
  - 60 more paired samples are averaged

Point 3:
  - test waits for another temperature rise while charging
  - final 60 paired samples are averaged

The Nano then fits internal raw ADC count against the external reference
temperature and gives exact replacement lines for PinsAndConfig.h:

  constexpr float INTERNAL_TEMP_CAL_RAW = ...f;
  constexpr float INTERNAL_TEMP_CAL_C = ...f;
  constexpr float INTERNAL_TEMP_COUNTS_PER_C = ...f;
  constexpr float INTERNAL_TEMP_CALIBRATION_OFFSET_C = 0.0f;

The temperature calibration stops automatically when complete. Reflash the
Nano with the returned values, then move the external sensor back to the battery.

Temperature-sync configuration in PinsAndConfig.h:

  TEMP_SYNC_SAMPLE_INTERVAL_MS
  TEMP_SYNC_SAMPLES_PER_POINT
  TEMP_SYNC_MIN_RISE_C
  TEMP_SYNC_MIN_TOTAL_SPAN_C

The internal ATmega temperature sensor is only a secondary enclosure/electronics
thermal safety channel. The external sensor remains the battery-temperature
safety sensor.

DEBUG / SERIAL MONITOR
----------------------
All USB Serial Monitor output is handled by Debug.h/.cpp.

PinsAndConfig.h controls:
  ENABLE_DEBUG
  DEBUG_BAUD
  DEBUG_INTERVAL_MS
  DEBUG_PRINT_STATE_CHANGES
  DEBUG_WAIT_FOR_SERIAL_MS

Default debug baud: 115200.

SAFETY
------
This firmware supervises the charger; it is not itself a certified multi-stage
lead-acid charger. Keep the current-limited charging stage and fuse in place.
Verify that the charger cutoff hardware really removes charging current before
relying on automatic shutdown.

The MOSFET/gate hardware is controlled only through gateway.h/.cpp. If the
power stage uses a low-side N-channel MOSFET, avoid accidentally bypassing the
switched negative path through Nano ground, USB ground, another supply, or test
equipment.
