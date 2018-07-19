#ifndef _TAG_IDS_HPP_
#define _TAG_IDS_HPP_

#include <stdint.h>
#include <string.h>
#include "ring_effects.hpp"

#define num_tags 3

struct tag
{
     uint8_t uid[7];
     RGB_VALS rgb;
     char name[32];
     char wav_file[32];
};

const tag tag0 = {{0,0,0,0,0,0,0}, {0,0,220}, "unknown", "test.wav"};
const tag tag1 = {{5,248,250,209,0,0,0}, {220,0,0}, "red", "red.wav"};
const tag tag2 = {{69,86,216,209,0,0,0}, {0,220,0}, "green", "test.wav"};
const struct tag tags[num_tags] = {tag0, tag1, tag2};

struct tag find_tag(uint8_t * uid);

#endif
