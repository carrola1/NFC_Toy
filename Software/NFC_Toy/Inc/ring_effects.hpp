#ifndef _RING_EFFECTS_H_
#define _RING_EFFECTS_H_

#include "dotstar.hpp"

struct RGB_VALS
{
   uint8_t r;
   uint8_t g;
   uint8_t b;
};

void loop_animation(DotStar& ring, uint8_t ring_size, uint8_t n_loops, RGB_VALS rgb);


#endif // _RING_EFFECTS_H_