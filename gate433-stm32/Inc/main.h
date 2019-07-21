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
#include "stm32f1xx_hal.h"

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
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define OPEN_Pin GPIO_PIN_12
#define OPEN_GPIO_Port GPIOB
#define LOOP_O_Pin GPIO_PIN_14
#define LOOP_O_GPIO_Port GPIOB
#define LOOP_I_Pin GPIO_PIN_15
#define LOOP_I_GPIO_Port GPIOB
#define BTN_CLK_Pin GPIO_PIN_15
#define BTN_CLK_GPIO_Port GPIOA
#define BTN_CLK_EXTI_IRQn EXTI15_10_IRQn
#define BTN_DATA_Pin GPIO_PIN_3
#define BTN_DATA_GPIO_Port GPIOB
#define BTN_SW_Pin GPIO_PIN_4
#define BTN_SW_GPIO_Port GPIOB
#define BTN_SW_EXTI_IRQn EXTI4_IRQn
#define BTN_PLUS_Pin GPIO_PIN_5
#define BTN_PLUS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
