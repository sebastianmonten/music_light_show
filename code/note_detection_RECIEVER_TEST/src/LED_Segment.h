#ifndef MY_LED_SEGMENT_H
#define MY_LED_SEGMENT_H


#include <Arduino.h>
#include <FastLED.h>

class LED_Segment {
  private:
    CRGB led_strip;
    byte num_leds;
    byte start_index;

  public:
    LED_Segment(CRGB led_strip[], byte num_leds, byte start_index);
    void fill_red();
};

#endif