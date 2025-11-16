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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;

// Interupt variables
#define UART_RX_BUFFER_LEN 256
extern uint8_t UART_RX_BUFFER[UART_RX_BUFFER_LEN];
extern osEventFlagsId_t respondToInputEventFlags;
extern TaskHandle_t xRxHandlerTask;


enum MessageIds {
	MESSAGE_ID_PING = 0,
};

extern uint8_t MESSAGE_HEADER[4];
#define PING_MESSAGE {MESSAGE_HEADER[0], MESSAGE_HEADER[1], MESSAGE_HEADER[2], MESSAGE_HEADER[3], 6, MESSAGE_ID_PING}
#define IS_VALID_MESSAGE(message, size) (size >= 6 && \
	message[0] == MESSAGE_HEADER[0] && \
	message[1] == MESSAGE_HEADER[1] && \
	message[2] == MESSAGE_HEADER[2] && \
	message[3] == MESSAGE_HEADER[3] && \
	message[4] == (uint8_t)size\
)

#define MESSAGE_TYPE(message) (message[5])
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define m0_ipropi_Pin GPIO_PIN_0
#define m0_ipropi_GPIO_Port GPIOA
#define m0_gainsel_Pin GPIO_PIN_1
#define m0_gainsel_GPIO_Port GPIOA
#define m0_pwm_f_Pin GPIO_PIN_2
#define m0_pwm_f_GPIO_Port GPIOA
#define m0_pwm_r_Pin GPIO_PIN_3
#define m0_pwm_r_GPIO_Port GPIOA
#define m1_ipropi_Pin GPIO_PIN_4
#define m1_ipropi_GPIO_Port GPIOA
#define m1_gainsel_Pin GPIO_PIN_5
#define m1_gainsel_GPIO_Port GPIOA
#define m1_pwm_f_Pin GPIO_PIN_6
#define m1_pwm_f_GPIO_Port GPIOA
#define m1_pwm_r_Pin GPIO_PIN_7
#define m1_pwm_r_GPIO_Port GPIOA
#define m2_ipropi_Pin GPIO_PIN_4
#define m2_ipropi_GPIO_Port GPIOC
#define m2_pwm_f_Pin GPIO_PIN_0
#define m2_pwm_f_GPIO_Port GPIOB
#define batt_voltage_Pin GPIO_PIN_1
#define batt_voltage_GPIO_Port GPIOB
#define batt_temp_Pin GPIO_PIN_2
#define batt_temp_GPIO_Port GPIOB
#define m2_gainsel_Pin GPIO_PIN_10
#define m2_gainsel_GPIO_Port GPIOB
#define m2_pwm_r_Pin GPIO_PIN_11
#define m2_pwm_r_GPIO_Port GPIOB
#define m3_ipropi_Pin GPIO_PIN_12
#define m3_ipropi_GPIO_Port GPIOB
#define m3_gainsel_Pin GPIO_PIN_13
#define m3_gainsel_GPIO_Port GPIOB
#define m3_pwm_f_Pin GPIO_PIN_14
#define m3_pwm_f_GPIO_Port GPIOB
#define m3_pwm_r_Pin GPIO_PIN_15
#define m3_pwm_r_GPIO_Port GPIOB
#define IMU_int_Pin GPIO_PIN_6
#define IMU_int_GPIO_Port GPIOC
#define vl53l1x_gpio1_Pin GPIO_PIN_11
#define vl53l1x_gpio1_GPIO_Port GPIOA
#define vl53l1x_xshut_Pin GPIO_PIN_12
#define vl53l1x_xshut_GPIO_Port GPIOA
#define debug_led_Pin GPIO_PIN_9
#define debug_led_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
