#ifndef EdSoftLED_H_
#define EdSoftLED_H_

#include <Arduino.h>

struct RGBW {  // Store of the original values
  uint8_t g;   // Green
  uint8_t r;   // Red
  uint8_t b;   // Blue
};

class EdSoftLED {
 public:
  EdSoftLED(uint16_t lamps, const uint8_t pin);
  ~EdSoftLED();

  void off(void);
  void update(uint16_t lamp, uint32_t RGBWColor);

 private:
  uint8_t _pin_mask;
  uint16_t _count_led;
  RGBW *_pixels;
  rmt_data_t *_LEDdata;
  uint8_t _pin_number;
  uint8_t getRed(uint32_t c);
  uint8_t getGreen(uint32_t c);
  uint8_t getBlue(uint32_t c);
  void fill(uint32_t RGBWColor, uint16_t FirstLed, uint16_t NoofLEDs);
  void sendData(void);
};

#endif /* EdSoftLED_H_ */
