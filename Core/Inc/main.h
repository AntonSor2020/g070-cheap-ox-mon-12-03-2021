/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_hal.h"

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
#define SegA_Pin GPIO_PIN_4
#define SegA_GPIO_Port GPIOA
#define SegF_Pin GPIO_PIN_5
#define SegF_GPIO_Port GPIOA
#define SegB_Pin GPIO_PIN_6
#define SegB_GPIO_Port GPIOA
#define SegG_Pin GPIO_PIN_7
#define SegG_GPIO_Port GPIOA
#define SegC_Pin GPIO_PIN_0
#define SegC_GPIO_Port GPIOB
#define SegDP_Pin GPIO_PIN_1
#define SegDP_GPIO_Port GPIOB
#define SegD_Pin GPIO_PIN_2
#define SegD_GPIO_Port GPIOB
#define SegE_Pin GPIO_PIN_8
#define SegE_GPIO_Port GPIOA
#define D_1_Pin GPIO_PIN_9
#define D_1_GPIO_Port GPIOA
#define D_2_Pin GPIO_PIN_6
#define D_2_GPIO_Port GPIOC
#define D_3_Pin GPIO_PIN_10
#define D_3_GPIO_Port GPIOA
#define D_4_Pin GPIO_PIN_11
#define D_4_GPIO_Port GPIOA
#define D_5_Pin GPIO_PIN_12
#define D_5_GPIO_Port GPIOA
#define D_6_Pin GPIO_PIN_15
#define D_6_GPIO_Port GPIOA
#define RedLed_Pin GPIO_PIN_3
#define RedLed_GPIO_Port GPIOB
#define IRLed_Pin GPIO_PIN_4
#define IRLed_GPIO_Port GPIOB
#define AmplPower_Pin GPIO_PIN_5
#define AmplPower_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
