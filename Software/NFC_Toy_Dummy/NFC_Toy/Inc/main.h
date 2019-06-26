/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AUDIO_SD_N_Pin GPIO_PIN_13
#define AUDIO_SD_N_GPIO_Port GPIOC
#define SD_DETECT_Pin GPIO_PIN_2
#define SD_DETECT_GPIO_Port GPIOA
#define SD_SW_Pin GPIO_PIN_3
#define SD_SW_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define DOT_STAR_DATA_Pin GPIO_PIN_0
#define DOT_STAR_DATA_GPIO_Port GPIOB
#define DOT_STAR_CLK_Pin GPIO_PIN_1
#define DOT_STAR_CLK_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_10
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_11
#define LED_2_GPIO_Port GPIOA
#define XL_INT1_Pin GPIO_PIN_15
#define XL_INT1_GPIO_Port GPIOA
#define XL_INT1_EXTI_IRQn EXTI4_15_IRQn
#define XL_INT2_Pin GPIO_PIN_3
#define XL_INT2_GPIO_Port GPIOB
#define NFC_RST_PDN_N_Pin GPIO_PIN_4
#define NFC_RST_PDN_N_GPIO_Port GPIOB
#define NFC_RST_N_Pin GPIO_PIN_8
#define NFC_RST_N_GPIO_Port GPIOB
#define NFC_IRQ_Pin GPIO_PIN_9
#define NFC_IRQ_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
