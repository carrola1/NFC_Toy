#ifndef WAV_PLAYER_H
#define WAV_PLAYER_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "stm32l0xx_hal.h"
#include "diskio.h"
#include "ff.h"
#include "i2s.h"

const uint16_t buf_size = 512;

uint8_t audio_samp[2];
uint16_t audio_buf_0[512];
uint16_t audio_buf_1[512];
uint16_t *ptr;
uint16_t *ptr_start;
uint16_t *ptr_end;
UINT bytes_read;

FRESULT fr;     /* FatFs return code */

void play_wav(void);


#ifdef __cplusplus
}
#endif
#endif
