#include "EdSoftLED.h"
// Create
EdSoftLED::EdSoftLED(uint16_t num_leds, const uint8_t pin) {
  _count_led = num_leds;
  _pin_number = pin;
  _pixels = new RGBW[_count_led];
  _LEDdata = new rmt_data_t[32 * _count_led];

  if (!rmtInit(_pin_number, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) {
    log_e("init sender failed");
  }
  log_e("real tick set to: 100ns");
}

// Close
EdSoftLED::~EdSoftLED() {
  delete[] _pixels;
  delete[] _LEDdata;
}

void EdSoftLED::off(void) {
    EdSoftLED::fill(0X00000000, 0, _count_led);
    EdSoftLED::sendData();
  // digitalWrite(_pin_number, LOW);
}

// Show WS2812 LED strip
void EdSoftLED::sendData() {
  uint32_t Kleur = 0;
  uint32_t LedDataBit = 0;  // counter for the bit in _LEDdata

  for (uint32_t i = 0; i < _count_led; i++) {  // Prepare the string with dimmed values
    const uint8_t r = _pixels[i].r;
    const uint8_t g = _pixels[i].g;
    const uint8_t b = _pixels[i].b;
    // WS2812 expects 24 bits sent MSB->LSB in the LED’s native byte order.
    // RGB: [R][G][B] ; GRB: [G][R][B]
    Kleur = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; // WS2812RGB

    for (uint8_t bit = 0; bit < 24; bit++) {
      if (Kleur & (1UL << (23 - bit))) {
        _LEDdata[LedDataBit].level0 = 1;
        _LEDdata[LedDataBit].duration0 = 6; // T1H 0.6 µs
        _LEDdata[LedDataBit].level1 = 0;
        _LEDdata[LedDataBit].duration1 = 6; // T1L 0.6 µs
      } else {
        _LEDdata[LedDataBit].level0 = 1;
        _LEDdata[LedDataBit].duration0 = 3; // T0H 0.3 µs
        _LEDdata[LedDataBit].level1 = 0;
        _LEDdata[LedDataBit].duration1 = 9; // T0L 0.9 µs
      }
      LedDataBit++;
    }
    Kleur = 0;
  }

  rmtWrite(_pin_number, _LEDdata, _count_led * 24, RMT_WAIT_FOR_EVER);
}

void EdSoftLED::update(uint16_t lamp, uint32_t RGBWColor) {
  if (lamp < _count_led) {
    _pixels[lamp].r = getRed(RGBWColor);
    _pixels[lamp].g = getGreen(RGBWColor);
    _pixels[lamp].b = getBlue(RGBWColor);
  }
  EdSoftLED::sendData();
}


// Fill part of the LED Strip with an RGBW color
void EdSoftLED::fill(uint32_t RGBWColor, uint16_t FirstLed, uint16_t NoofLEDs) {
  uint16_t LastLED = FirstLed + NoofLEDs;
  if (LastLED > _count_led) LastLED = _count_led;
  for (uint16_t i = FirstLed; i < LastLED; i++) {
    _pixels[i].r = getRed(RGBWColor);
    _pixels[i].g = getGreen(RGBWColor);
    _pixels[i].b = getBlue(RGBWColor);
  }
}

// Get Red color
uint8_t EdSoftLED::getRed(uint32_t c) { return (c >> 8); }
// Get Green color
uint8_t EdSoftLED::getGreen(uint32_t c) { return (c >> 16); }
// Get Blue color
uint8_t EdSoftLED::getBlue(uint32_t c) { return (c); }
