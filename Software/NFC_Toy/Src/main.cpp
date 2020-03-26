
/**
  ******************************************************************************
  * @file           : mainc
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners
  *
  * Copyright (c) 2018 STMicroelectronics International NV
  * All rights reserved
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1 Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer
  * 2 Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution
  * 3 Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission
  * 4 This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics
  * 5 Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "dma.h"
#include "diskio.h"
#include "ff.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "gpio.h"
#include "rng.h"
#include "dotstar.hpp"
#include "ring_effects.hpp"
#include "lis3dh.hpp"
#include "wav_player.h"
#include "pn532.h"
#include "tag_ids.hpp"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS FatFs;

// Tasks
typedef struct task {
  uint16_t period;                // Rate at which the task should tick
  uint16_t elapsedTime;           // Time since task's last tick
  uint8_t state;                  //Current state
  uint8_t (*TickFct)(uint8_t);  // Function to call for task's tick
} task;

enum NFC_States { ST_nfc_ping, ST_nfc_status };
uint8_t TickFct_nfc(uint8_t);

enum LED_States { ST_led_idle, ST_led_off, ST_led_ring, ST_led_on_tag_detected, ST_led_on_request };
uint8_t TickFct_led(uint8_t);

enum Audio_States { ST_audio_idle, ST_audio_play_song, ST_audio_play_request };
uint8_t TickFct_audio(uint8_t);

uint8_t timer_flag = 0;
uint8_t tasks_timer = 0;
uint16_t tasks_period = 50; // Timer tick rate

// Create objects
LIS3DH accel = LIS3DH();
DotStar ring = DotStar(16, DOTSTAR_RBG);
PN532 nfc = PN532();

// Global Settings/Flags
uint8_t brightness_all_on = 5;
uint8_t brightness_ring = 20;
RGB_VALS rgb_off;
bool nfc_found;
bool correct_tag_found;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
tag detected_tag;
tag tag_to_find;
uint8_t randtagid;
uint8_t nfc_wait_timer = 0;
uint8_t nfc_wait_timeout = 3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  // Setup accelerometer
  accel.begin(0x32);
  accel.writeRegister8(LIS3DH_REG_INT1CFG, 0x2A);
  accel.writeRegister8(LIS3DH_REG_INT1THS, 0xBF);
  accel.writeRegister8(LIS3DH_REG_INT1DUR, 0x30);
  accel.writeRegister8(LIS3DH_REG_CTRL3, 0x40);

  // Setup and initialize Dotstars
  ring.begin(); // Initialize pins for output
  rgb_off.r = 0; rgb_off.g = 0; rgb_off.b = 0;
  ring_set_all_pixels(ring, rgb_off); // Initialize LEDs to off

  // Mount SD Card
  FRESULT fr;     /* FatFs return code */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  fr = f_mount(&FatFs, "", 1);

  // Setup NFC Reader
  HAL_GPIO_WritePin(NFC_RST_PDN_N_GPIO_Port, NFC_RST_PDN_N_Pin, GPIO_PIN_SET);
  nfc.begin();
  nfc.SAMConfig();
  randtagid = HAL_RNG_GetRandomNumber(&hrng) % num_tags;
  tag_to_find = tags[randtagid];

  // Configure tasks
  task tasks[3];
  const uint8_t tasks_num = 3;
  const uint16_t period_nfc   = 200;
  const uint16_t period_led   = 50;
  const uint16_t period_audio = 200;

  tasks[0].period      = period_nfc;
  tasks[0].elapsedTime = tasks[0].period;
  tasks[0].TickFct     = &TickFct_nfc;
  tasks[0].state       = ST_nfc_ping;

  tasks[1].period      = period_led;
  tasks[1].elapsedTime = tasks[1].period;
  tasks[1].TickFct     = &TickFct_led;
  tasks[1].state       = ST_led_ring;
  
  tasks[2].period      = period_audio;
  tasks[2].elapsedTime = tasks[2].period;
  tasks[2].TickFct     = &TickFct_audio;
  tasks[2].state       = ST_audio_play_request;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Task Scheduler
  while(1) {
      // Heart of the scheduler code
      for (uint8_t i = 0; i < tasks_num; ++i) {
        if (tasks[i].elapsedTime >= tasks[i].period) { // Ready
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += tasks_period;
      }
      timer_flag = 0;
      while (!timer_flag) {
        // wait for timeout
      }
  }
  return 0;
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  
    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void) {
  if (tasks_timer == tasks_period) {
    timer_flag = 1;
    tasks_timer = 0;
  } else {
    tasks_timer++;
  }
  return;
}

// NFC State Machine
uint8_t TickFct_nfc(uint8_t state) {
  switch(state) { // Transitions
    case ST_nfc_ping:
      state = ST_nfc_status;
      break;
    case ST_nfc_status:
      if (nfc_found) {
        state = ST_nfc_ping;
      } else if (nfc_wait_timer == nfc_wait_timeout) {
        state = ST_nfc_ping;
      } else {
        state = ST_nfc_status;
      }
      break;
    default:
      state = ST_nfc_ping;
  }

  switch(state) { // State actions
    case ST_nfc_ping:
      nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, 10);
      nfc_wait_timer = 0;
      break;
    case ST_nfc_status:
      nfc_found = nfc.checkPassiveTargetStatus(uid, &uidLength);
      if (nfc_found) {
        detected_tag = find_tag(uid);
        correct_tag_found = (detected_tag.uid[0] == tag_to_find.uid[0]);
        randtagid = HAL_RNG_GetRandomNumber(&hrng) % num_tags;
        tag_to_find = tags[randtagid];
      }
      nfc_wait_timer++;
      break;
    default:
      break;
  }
  
  return state;
}

// LED State Machine
uint8_t TickFct_led(uint8_t state) {
  switch(state) { // Transitions
    case ST_led_idle:
      if (nfc_found == 0) {
        state = ST_led_ring;
      } else {
        state = ST_led_idle;
      }
      break;
    case ST_led_off:
      state = ST_led_idle;
      break;
    case ST_led_ring:
      if (nfc_found) {
        state = ST_led_on_tag_detected;
      } else {
        state = ST_led_ring;
      }
      break;
    case ST_led_on_tag_detected:
      state = ST_led_on_request;
      break;
    case ST_led_on_request:
      state = ST_led_idle;
      break;
    default:
      state = ST_led_idle;
  }

  switch(state) { // State actions
    case ST_led_idle:
      // do nothing
      break;
    case ST_led_off:
      ring_set_all_pixels(ring, rgb_off);
      break;
    case ST_led_ring:
      ring.decrRing(tag_to_find.rgb);
      break;
    case ST_led_on_tag_detected:
      ring.setBrightness(brightness_all_on);
      ring_set_all_pixels(ring, detected_tag.rgb);
      ring.setBrightness(brightness_ring);
    case ST_led_on_request:
      ring.setBrightness(brightness_all_on);
      ring_set_all_pixels(ring, tag_to_find.rgb);
      ring.setBrightness(brightness_ring);
    default:
      break;
  }
  
  return state;
}

// Audio State Machine
uint8_t TickFct_audio(uint8_t state) {
  switch(state) { // Transitions
    case ST_audio_idle:
      if (nfc_found) {
        state = ST_audio_play_song;
      } else {
        state = ST_audio_idle;
      }
      break;
    case ST_audio_play_song:
      state = ST_audio_play_request;
      break;
    case ST_audio_play_request:
      state = ST_audio_idle;
      break;
    default:
      state = ST_audio_idle;
  }
  switch(state) { // State actions
    case ST_audio_idle:
      // do nothing
      break;
    case ST_audio_play_song:
      if (correct_tag_found) {
        play_wav(detected_tag.wav_file_found);
      }
      play_wav(detected_tag.wav_file_color);
      break;
    case ST_audio_play_request:
      play_wav(tag_to_find.wav_file_find);
      nfc_found = 0;
      break;
    default:
      break;
  }
  return state;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence
  * @param  file: The file name as string
  * @param  line: The line in file as a number
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
