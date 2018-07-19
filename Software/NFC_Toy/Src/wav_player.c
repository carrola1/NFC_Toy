#include "wav_player.h"

void play_wav(void) {
    HAL_GPIO_WritePin(AUDIO_SD_N_GPIO_Port, AUDIO_SD_N_Pin, GPIO_PIN_SET);
    FIL fil;                                // Create file object
    fr = f_open(&fil, "red.wav", FA_READ);  // open file
    //f_lseek(&fil, 76);                      // move to data region of .wav
    ptr = audio_buf_0;        // point to buffer 0 first
    ptr_start = audio_buf_0;

    while(1) {
        f_read(&fil, &wav_buf[0], 512, &bytes_read);

        //////////// End of File ////////////
        if (bytes_read < 2) {
            f_close(&fil);
            HAL_GPIO_WritePin(AUDIO_SD_N_GPIO_Port, AUDIO_SD_N_Pin, GPIO_PIN_SET);
            return;
        }
        //////////// End of File ////////////

        // covert raw bytes from wav file into 16-bit audio samples
        for (int ii=0; ii<511; ii+=2) {
            *ptr = ((uint16_t)wav_buf[ii+1] << 8) | (uint16_t)wav_buf[ii];
            if (*ptr > 32767) {
            	*ptr = (*ptr >> 1) + 32768;
            } else {
            	*ptr = *ptr >> 1;
            }
            ptr++;
        }

        while (hi2s2.State != HAL_I2S_STATE_READY);    // Wait for I2S to be ready
        HAL_I2S_Transmit_DMA(&hi2s2, ptr_start, buf_size);    // play buffer
        
        // ping pong buffer
        if (ptr_start == &audio_buf_0[0]) {
            ptr = &audio_buf_1[0];
            ptr_start = &audio_buf_1[0];
        } else {
            ptr = &audio_buf_0[0];
            ptr_start = &audio_buf_0[0];
        }
    }
    
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s) {
    return;
}
