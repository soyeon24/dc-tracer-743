/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define E3_Pin GPIO_PIN_3
#define E3_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOE
#define FAN_Pin GPIO_PIN_6
#define FAN_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define L_IPROPR_Pin GPIO_PIN_2
#define L_IPROPR_GPIO_Port GPIOC
#define R_IPROPR_Pin GPIO_PIN_3
#define R_IPROPR_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define Sensor_MUX4_Pin GPIO_PIN_2
#define Sensor_MUX4_GPIO_Port GPIOA
#define Sensor_MUX3_Pin GPIO_PIN_3
#define Sensor_MUX3_GPIO_Port GPIOA
#define Sensor_MUX2_Pin GPIO_PIN_4
#define Sensor_MUX2_GPIO_Port GPIOA
#define Sensor_MUX1_Pin GPIO_PIN_5
#define Sensor_MUX1_GPIO_Port GPIOA
#define Sensor_MUX0_Pin GPIO_PIN_6
#define Sensor_MUX0_GPIO_Port GPIOA
#define Sensor_ADC_Pin GPIO_PIN_7
#define Sensor_ADC_GPIO_Port GPIOA
#define LED2C4_Pin GPIO_PIN_4
#define LED2C4_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LCD_WR_RS_Pin GPIO_PIN_13
#define LCD_WR_RS_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define L_nFAULT_Pin GPIO_PIN_10
#define L_nFAULT_GPIO_Port GPIOD
#define L_DRVOFF_Pin GPIO_PIN_13
#define L_DRVOFF_GPIO_Port GPIOD
#define L_DIR_Pin GPIO_PIN_14
#define L_DIR_GPIO_Port GPIOD
#define L_nSLEEP_Pin GPIO_PIN_15
#define L_nSLEEP_GPIO_Port GPIOD
#define R_PWM_Pin GPIO_PIN_6
#define R_PWM_GPIO_Port GPIOC
#define L_PWM_Pin GPIO_PIN_7
#define L_PWM_GPIO_Port GPIOC
#define R_DRVOFF_Pin GPIO_PIN_8
#define R_DRVOFF_GPIO_Port GPIOA
#define R_DIR_Pin GPIO_PIN_9
#define R_DIR_GPIO_Port GPIOA
#define R_nSLEEP_Pin GPIO_PIN_10
#define R_nSLEEP_GPIO_Port GPIOA
#define R_nFAULT_Pin GPIO_PIN_15
#define R_nFAULT_GPIO_Port GPIOA
#define SWU_Pin GPIO_PIN_1
#define SWU_GPIO_Port GPIOD
#define SWL_Pin GPIO_PIN_3
#define SWL_GPIO_Port GPIOD
#define SWR_Pin GPIO_PIN_4
#define SWR_GPIO_Port GPIOD
#define SWD_Pin GPIO_PIN_5
#define SWD_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
