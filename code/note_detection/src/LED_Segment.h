#include <Arduino.h>
#include <FastLED.h>

class LED_Segment {
  public:
  void Begin(CRGB [12], int, int);
  
  private:
  CRGB _led_strip[12];
  int _start_index;
  int _num_led;
};  