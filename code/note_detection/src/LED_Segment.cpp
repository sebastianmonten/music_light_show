#include "LED_Segment.h"

LED_Segment::LED_Segment(CRGB led_strip[], byte num_leds, byte start_index) {
    // attempt 1:
    // this->led_strip = led_strip;

    // attempt 2:
    // for (int i = 0; i < num_leds; i++) {
    //     *this->led_strip[i] = *led_strip[i];
    // }

    // attempt 3:
    // memcpy(this->led_strip, led_strip, num_leds);

    //attmpt 4:
    for (int i = 0; i < num_leds; i++) {
        this->led_strip[i] = led_strip[i];
    }

    this->num_leds = num_leds;
    this->start_index = start_index;
}