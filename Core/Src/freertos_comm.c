/*
 * freertos_comm.c
 *
 *  Created on: Mar 29, 2026
 *      Author: tomwolcott
 */

#include "freertos_comm.h"
#include "freertos.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

EventGroupHandle_t xI2CEventGroup;

void i2c_interrupt_handler_tx(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult = pdFAIL;

	if (hi2c->Instance == hi2c1.Instance) {
		xResult = xEventGroupSetBitsFromISR(xI2CEventGroup, FREERTOS_COMM_I2C1_TX_EVENT, &xHigherPriorityTaskWoken);
	} else if (hi2c->Instance == hi2c2.Instance) {
		xResult = xEventGroupSetBitsFromISR(xI2CEventGroup, FREERTOS_COMM_I2C2_TX_EVENT, &xHigherPriorityTaskWoken);
	}

	if( xResult != pdFAIL ) {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void i2c_interrupt_handler_rx(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken;
	BaseType_t xResult = pdFAIL;

	if (hi2c->Instance == hi2c1.Instance) {
		xResult = xEventGroupSetBitsFromISR(xI2CEventGroup, FREERTOS_COMM_I2C1_RX_EVENT, &xHigherPriorityTaskWoken);
	} else if (hi2c->Instance == hi2c2.Instance) {
		xResult = xEventGroupSetBitsFromISR(xI2CEventGroup, FREERTOS_COMM_I2C2_RX_EVENT, &xHigherPriorityTaskWoken);
	}

	if( xResult != pdFAIL ) {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void init_freertos_i2c() {
	for (int i = 0; i < 20; i++) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		HAL_Delay(200);
	}

	xI2CEventGroup = xEventGroupCreate();
	HAL_Delay(1000);

	for (int i = 0; i < 20; i++) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		HAL_Delay(200);
	}
}

HAL_StatusTypeDef i2c_write_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size) {
	EventBits_t event_bits = (settings->hi2c->Instance == hi2c1.Instance) ? FREERTOS_COMM_I2C1_EVENT : FREERTOS_COMM_I2C2_EVENT;
    EventBits_t uxBits = xEventGroupWaitBits(
    	xI2CEventGroup,
		event_bits,
        pdTRUE, pdFALSE, portMAX_DELAY
    );

    if (event_bits & uxBits == 0) {
    	return HAL_ERROR;
    }

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(
	  settings->hi2c,
	  settings->address,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  data, size
	);

	return status;
}

HAL_StatusTypeDef i2c_write_register(i2cSettings *settings, uint8_t reg, uint8_t data) {
	return i2c_write_registers(settings, reg, &data, 1);
}

HAL_StatusTypeDef i2c_read_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size) {
	EventBits_t event_bits = (settings->hi2c->Instance == hi2c1.Instance) ? FREERTOS_COMM_I2C1_EVENT : FREERTOS_COMM_I2C2_EVENT;
    EventBits_t uxBits = xEventGroupWaitBits(
    	xI2CEventGroup,
		event_bits,
        pdTRUE, pdFALSE, portMAX_DELAY
    );

    if (event_bits & uxBits == 0) {
    	return HAL_ERROR;
    }

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(
	  settings->hi2c,
	  settings->address,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  data, size
	);

	return status;
}

uint8_t i2c_read_register(i2cSettings *settings, uint8_t reg) {
	uint8_t data;
	i2c_read_registers(settings, reg, &data, 1);
	return data;
}

uint8_t i2c_read_register_with_code(i2cSettings *settings, uint8_t reg, HAL_StatusTypeDef *error_code) {
	uint8_t data;
	*error_code = i2c_read_registers(settings, reg, &data, 1);
	return data;
}
