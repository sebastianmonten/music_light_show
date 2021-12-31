#include <Arduino.h>
#include <FastLED.h>

#define PIN_C 13
#define NUM_LEDS_C 60

#define PIN_Db_D_Eb 12
#define NUM_LEDS_Db_D_Eb 47


#define START_C 0
#define START_Db 22
#define START_D 12
#define START_Eb 0




CRGB leds_C[NUM_LEDS_C];
CRGB leds_Db_D_Eb[NUM_LEDS_Db_D_Eb];
 


void setup() {
  FastLED.addLeds<WS2812B, PIN_C, GRB>(leds_C, NUM_LEDS_C);
  FastLED.addLeds<WS2812B, PIN_C, GRB>(leds_Db_D_Eb, NUM_LEDS_Db_D_Eb);
  FastLED.setBrightness(100);
}

void loop() {
  for (int i = START_D; i < START_Db; i++) {
     leds_Db_D_Eb[i] = CRGB::Red;
     FastLED.show();
     delay(10);
     leds_Db_D_Eb[i] = CRGB::Black;
  }
  for (int i = START_Db-1; i >= START_D; i--) {
      leds_Db_D_Eb[i] = CRGB::Red;
      FastLED.show();
      delay(10);
      leds_Db_D_Eb[i] = CRGB::Black;
  }
  FastLED.show();
}
