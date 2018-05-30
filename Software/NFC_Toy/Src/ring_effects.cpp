#include "ring_effects.hpp"
#include "stm32l0xx_hal.h"

void loop_animation(DotStar& ring, uint8_t ring_size, uint8_t n_loops, RGB_VALS rgb) {
    
    for (uint8_t i = 0; i < n_loops; i++) 
    {
        for(uint8_t j = ring_size-1; j >= 0; j--)
        {
            ring.setPixelColor(j, rgb.r, rgb.g, rgb.b);
            if (i == ring_size-1) {
                ring.setPixelColor(0, 0, 0, 0);
            } else {
                ring.setPixelColor(j+1, rgb.r, rgb.g, rgb.b);
            }
            ring.show();
            HAL_Delay(100);
        }
    }
    return;
}