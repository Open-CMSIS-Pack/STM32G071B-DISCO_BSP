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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_ucpd.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

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
extern int stdio_init   (void);
extern int app_main     (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD6_SINK_SPY_Pin GPIO_PIN_12
#define LD6_SINK_SPY_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_2
#define B1_GPIO_Port GPIOF
#define JOY_SEL_Pin GPIO_PIN_0
#define JOY_SEL_GPIO_Port GPIOC
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOC
#define JOY_DOWN_Pin GPIO_PIN_2
#define JOY_DOWN_GPIO_Port GPIOC
#define JOY_RIGHT_Pin GPIO_PIN_3
#define JOY_RIGHT_GPIO_Port GPIOC
#define EN_SMPS_Pin GPIO_PIN_0
#define EN_SMPS_GPIO_Port GPIOA
#define CS_DISP_Pin GPIO_PIN_3
#define CS_DISP_GPIO_Port GPIOA
#define RST_DISP_Pin GPIO_PIN_6
#define RST_DISP_GPIO_Port GPIOA
#define D_C_DISP_Pin GPIO_PIN_7
#define D_C_DISP_GPIO_Port GPIOA
#define JOY_UP_Pin GPIO_PIN_4
#define JOY_UP_GPIO_Port GPIOC
#define ALERT_VBUS_Pin GPIO_PIN_5
#define ALERT_VBUS_GPIO_Port GPIOC
#define ENCC1_Pin GPIO_PIN_10
#define ENCC1_GPIO_Port GPIOB
#define ENCC2_Pin GPIO_PIN_11
#define ENCC2_GPIO_Port GPIOB
#define RD_CC1_Pin GPIO_PIN_12
#define RD_CC1_GPIO_Port GPIOB
#define ALERT_CC1_Pin GPIO_PIN_6
#define ALERT_CC1_GPIO_Port GPIOC
#define ALERT_CC2_Pin GPIO_PIN_7
#define ALERT_CC2_GPIO_Port GPIOC
#define LD5_TO_PLUG_Pin GPIO_PIN_8
#define LD5_TO_PLUG_GPIO_Port GPIOD
#define LD4_TO_REC_Pin GPIO_PIN_9
#define LD4_TO_REC_GPIO_Port GPIOD
#define STLK_ON_Pin GPIO_PIN_11
#define STLK_ON_GPIO_Port GPIOA
#define SMPS_ON_Pin GPIO_PIN_12
#define SMPS_ON_GPIO_Port GPIOA
#define LD7_SOURCE_Pin GPIO_PIN_5
#define LD7_SOURCE_GPIO_Port GPIOD
#define DOOR_SENSOR_Pin GPIO_PIN_6
#define DOOR_SENSOR_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
