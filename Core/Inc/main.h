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
#include "stm32f4xx_hal.h"

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
#define BUZZER_OUT_Pin GPIO_PIN_13
#define BUZZER_OUT_GPIO_Port GPIOC
#define BUTTON_GPIO_Pin GPIO_PIN_0
#define BUTTON_GPIO_GPIO_Port GPIOA
#define BUTTON_GPIO_EXTI_IRQn EXTI0_IRQn
#define UART_RADIO_TX_Pin GPIO_PIN_2
#define UART_RADIO_TX_GPIO_Port GPIOA
#define UART_RADIO_RX_Pin GPIO_PIN_3
#define UART_RADIO_RX_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define TIM_MOTOR1_Pin GPIO_PIN_8
#define TIM_MOTOR1_GPIO_Port GPIOA
#define TIM_MOTOR2_Pin GPIO_PIN_9
#define TIM_MOTOR2_GPIO_Port GPIOA
#define TIM_MOTOR3_Pin GPIO_PIN_10
#define TIM_MOTOR3_GPIO_Port GPIOA
#define TIM_MOTOR4_Pin GPIO_PIN_11
#define TIM_MOTOR4_GPIO_Port GPIOA
#define UART_RPI_TX_Pin GPIO_PIN_15
#define UART_RPI_TX_GPIO_Port GPIOA
#define UART_RPI_RX_Pin GPIO_PIN_3
#define UART_RPI_RX_GPIO_Port GPIOB
#define TIM_TASK_CH1_Pin GPIO_PIN_4
#define TIM_TASK_CH1_GPIO_Port GPIOB
#define I2C1_INT_Pin GPIO_PIN_8
#define I2C1_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
