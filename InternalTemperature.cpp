#include "InternalTemperature.h"
#include "PinsAndConfig.h"

InternalTemperature::InternalTemperature()
  : _temperatureC(NAN),
    _rawAdc(0),
    _valid(false),
    _lastReadMs(0) {
}

void InternalTemperature::begin() {
  _temperatureC = readCelsius();
  _valid = !isnan(_temperatureC) &&
           (_rawAdc >= INTERNAL_TEMP_RAW_MIN_VALID) &&
           (_rawAdc <= INTERNAL_TEMP_RAW_MAX_VALID) &&
           (_temperatureC >= INTERNAL_TEMP_MIN_VALID_C) &&
           (_temperatureC <= INTERNAL_TEMP_MAX_VALID_C);
  _lastReadMs = millis();
}

void InternalTemperature::update(unsigned long nowMs) {
  if ((nowMs - _lastReadMs) < INTERNAL_TEMP_INTERVAL_MS) {
    return;
  }

  _lastReadMs = nowMs;
  _temperatureC = readCelsius();
  _valid = !isnan(_temperatureC) &&
           (_rawAdc >= INTERNAL_TEMP_RAW_MIN_VALID) &&
           (_rawAdc <= INTERNAL_TEMP_RAW_MAX_VALID) &&
           (_temperatureC >= INTERNAL_TEMP_MIN_VALID_C) &&
           (_temperatureC <= INTERNAL_TEMP_MAX_VALID_C);
}

float InternalTemperature::readCelsius() {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // Save ADC configuration because A0 is also used for battery-voltage sensing.
  const uint8_t oldADMUX = ADMUX;
  const uint8_t oldADCSRA = ADCSRA;
  #if defined(ADCSRB)
    const uint8_t oldADCSRB = ADCSRB;
  #endif

  // Enable ADC and select the ATmega328P internal temperature sensor (ADC8)
  // using the internal 1.1 V reference.
  ADCSRA |= _BV(ADEN);

  // Ensure any extended mux bit is cleared before selecting ADC8.
  #if defined(ADCSRB) && defined(MUX5)
    ADCSRB &= ~_BV(MUX5);
  #endif

  // REFS1:0 = 11 -> internal 1.1 V reference
  // MUX3:0  = 1000 -> internal temperature sensor
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);

  // Allow the internal reference / mux to settle.
  delay(5);

  // Discard several conversions after changing reference/channel.
  for (uint8_t i = 0; i < 4; ++i) {
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC)) {}
    (void)ADCW;
  }

  uint32_t total = 0;
  for (uint8_t i = 0; i < INTERNAL_TEMP_ADC_SAMPLES; ++i) {
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC)) {}
    total += ADCW;
  }

  _rawAdc = static_cast<uint16_t>(total / INTERNAL_TEMP_ADC_SAMPLES);

  // Restore ADC registers for the normal battery-voltage analogRead path.
  ADMUX = oldADMUX;
  #if defined(ADCSRB)
    ADCSRB = oldADCSRB;
  #endif
  ADCSRA = oldADCSRA;

  if (!INTERNAL_TEMP_USE_RAW_CALIBRATION || INTERNAL_TEMP_COUNTS_PER_C == 0.0f) {
    return NAN;
  }

  // Per-board one-point raw calibration. The absolute offset varies substantially
  // between ATmega328P chips, so use this Nano's observed raw value as the
  // reference point. The slope is approximately 0.93 ADC count per degree C
  // with the internal 1.1 V reference.
  return INTERNAL_TEMP_CAL_C +
         ((static_cast<float>(_rawAdc) - INTERNAL_TEMP_CAL_RAW) /
          INTERNAL_TEMP_COUNTS_PER_C) +
         INTERNAL_TEMP_CALIBRATION_OFFSET_C;
#else
  _rawAdc = 0;
  return NAN;
#endif
}

bool InternalTemperature::valid() const {
  return _valid;
}

float InternalTemperature::celsius() const {
  return _temperatureC;
}

uint16_t InternalTemperature::rawAdc() const {
  return _rawAdc;
}
