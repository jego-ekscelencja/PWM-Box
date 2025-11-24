/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define TP_1_Pin GPIO_PIN_3
#define TP_1_GPIO_Port GPIOA
#define TP_2_Pin GPIO_PIN_4
#define TP_2_GPIO_Port GPIOA
#define TP_3_Pin GPIO_PIN_5
#define TP_3_GPIO_Port GPIOA
#define JEdge_3_Pin GPIO_PIN_6
#define JEdge_3_GPIO_Port GPIOA
#define JEdge_2_Pin GPIO_PIN_7
#define JEdge_2_GPIO_Port GPIOA
#define JEdge_1_Pin GPIO_PIN_0
#define JEdge_1_GPIO_Port GPIOB
#define JPWM_3_Pin GPIO_PIN_13
#define JPWM_3_GPIO_Port GPIOB
#define JPWM_2_Pin GPIO_PIN_14
#define JPWM_2_GPIO_Port GPIOB
#define JPWM_1_Pin GPIO_PIN_15
#define JPWM_1_GPIO_Port GPIOB
#define Fset5_Pin GPIO_PIN_15
#define Fset5_GPIO_Port GPIOA
#define Fset4_Pin GPIO_PIN_3
#define Fset4_GPIO_Port GPIOB
#define Fset3_Pin GPIO_PIN_4
#define Fset3_GPIO_Port GPIOB
#define Fset2_Pin GPIO_PIN_5
#define Fset2_GPIO_Port GPIOB
#define Fset1_Pin GPIO_PIN_6
#define Fset1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
