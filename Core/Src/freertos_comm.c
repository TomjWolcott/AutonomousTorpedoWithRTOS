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

//static EventGroupHandle_t xI2CEventGroup;
static SemaphoreHandle_t xI2C1Done;
static SemaphoreHandle_t xI2C2Done;

static SemaphoreHandle_t xI2C1Mutex;
static SemaphoreHandle_t xI2C2Mutex;

#define MAX_I2C_MUTEX_DELAY (100)
#define MAX_I2C_DONE_DELAY (50)
#define MAX_I2C_BLOCKING_DELAY (100)

void i2c_interrupt_handler_tx(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hi2c->Instance == I2C1) {
	    xSemaphoreGiveFromISR( xI2C1Done, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	} else if (hi2c->Instance == I2C2) {
	    xSemaphoreGiveFromISR( xI2C2Done, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void i2c_interrupt_handler_rx(I2C_HandleTypeDef *hi2c) {
	i2c_interrupt_handler_tx(hi2c); // these are all currently the same
}

void i2c_interrupt_handler_read(I2C_HandleTypeDef *hi2c) {
	i2c_interrupt_handler_tx(hi2c); // these are all currently the same
}

void i2c_interrupt_handler_write(I2C_HandleTypeDef *hi2c) {
	i2c_interrupt_handler_tx(hi2c); // these are all currently the same
}

/// TODO: Do something extra here
void i2c_interrupt_handler_error(I2C_HandleTypeDef *hi2c) {
	i2c_interrupt_handler_tx(hi2c); // these are all currently the same
}

void init_freertos_i2c() {
	xI2C1Done = xSemaphoreCreateBinary();
	xI2C2Done = xSemaphoreCreateBinary();

	xI2C1Mutex = xSemaphoreCreateMutex();
	xI2C2Mutex = xSemaphoreCreateMutex();
}

void i2c_flush(I2C_HandleTypeDef *hi2c) {
	SemaphoreHandle_t i2c_mutex = (hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
	uint8_t data = 1;
	xSemaphoreTake(i2c_mutex, 0);
	HAL_I2C_Mem_Write(hi2c, 0x55, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, MAX_I2C_BLOCKING_DELAY);
	xSemaphoreGive(i2c_mutex);
}

HAL_StatusTypeDef i2c_write_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size) {
	BaseType_t result = pdTRUE;
	SemaphoreHandle_t i2c_mutex = (settings->hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
	SemaphoreHandle_t i2c_done = (settings->hi2c->Instance == I2C1) ? xI2C1Done : xI2C2Done;

	if (xSemaphoreTake(i2c_mutex, MAX_I2C_MUTEX_DELAY) != pdTRUE) {
		return HAL_TIMEOUT;
	}
//	xSemaphoreTake(i2c_done, 0);

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
	  settings->hi2c,
	  settings->address,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  data, size,
	  MAX_I2C_BLOCKING_DELAY
	);

	if (status != HAL_OK) {
		printf("i2c_write err: %d\n", status);
	}

//	if (status == HAL_OK) {
//		result = xSemaphoreTake(i2c_done, MAX_I2C_DONE_DELAY);
//	}

	xSemaphoreGive(i2c_mutex);

	return (result == pdTRUE) ? status : HAL_TIMEOUT;
}

HAL_StatusTypeDef i2c_read_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size) {
	BaseType_t result = pdTRUE;
	SemaphoreHandle_t i2c_mutex = (settings->hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
	SemaphoreHandle_t i2c_done = (settings->hi2c->Instance == I2C1) ? xI2C1Done : xI2C2Done;

	if (xSemaphoreTake(i2c_mutex, MAX_I2C_MUTEX_DELAY) != pdTRUE) {
		return HAL_TIMEOUT;
	}
//	xSemaphoreTake(i2c_done, 0);

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
	  settings->hi2c,
	  settings->address,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  data, size,
	  MAX_I2C_BLOCKING_DELAY
	);

	if (status != HAL_OK) {
		printf("i2c_read err: %d\n", status);
	}

//	if (status == HAL_OK) {
//		result = xSemaphoreTake(i2c_done, MAX_I2C_DONE_DELAY);
//	}

	xSemaphoreGive(i2c_mutex);

	return (result == pdTRUE) ? status : HAL_TIMEOUT;
}

HAL_StatusTypeDef i2c_transmit(i2cSettings *settings, uint8_t *data, uint16_t size) {
	BaseType_t result = pdTRUE;
	SemaphoreHandle_t i2c_mutex = (settings->hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
	SemaphoreHandle_t i2c_done = (settings->hi2c->Instance == I2C1) ? xI2C1Done : xI2C2Done;

	if (xSemaphoreTake(i2c_mutex, MAX_I2C_MUTEX_DELAY) != pdTRUE) {
		return HAL_TIMEOUT;
	}
//	xSemaphoreTake(i2c_done, 0);

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
		settings->hi2c,
		settings->address,
		data, size,
		MAX_I2C_BLOCKING_DELAY
	);

	if (status != HAL_OK) {
		printf("i2c_transmit err: %d\n", status);
	}

//	if (status == HAL_OK) {
//		result = xSemaphoreTake(i2c_done, MAX_I2C_DONE_DELAY);
//	}

	xSemaphoreGive(i2c_mutex);

	return (result == pdTRUE) ? status : HAL_TIMEOUT;
}

HAL_StatusTypeDef i2c_receive(i2cSettings *settings, uint8_t *data, uint16_t size) {
	BaseType_t result = pdTRUE;
	SemaphoreHandle_t i2c_mutex = (settings->hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
	SemaphoreHandle_t i2c_done = (settings->hi2c->Instance == I2C1) ? xI2C1Done : xI2C2Done;

	if (xSemaphoreTake(i2c_mutex, MAX_I2C_MUTEX_DELAY) != pdTRUE) {
		return HAL_TIMEOUT;
	}
//	xSemaphoreTake(i2c_done, 0);

	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
		settings->hi2c,
		settings->address,
		data, size,
		MAX_I2C_BLOCKING_DELAY
	);

	if (status != HAL_OK) {
		printf("i2c_receive err: %d\n", status);
	}

//	if (status == HAL_OK) {
//		result = xSemaphoreTake(i2c_done, MAX_I2C_DONE_DELAY);
//	}

	xSemaphoreGive(i2c_mutex);

	return (result == pdTRUE) ? status : HAL_TIMEOUT;
}

//HAL_StatusTypeDef i2c_write_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size) {
//	SemaphoreHandle_t i2c_mutex = (settings->hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
//	SemaphoreHandle_t i2c_done = (settings->hi2c->Instance == I2C1) ? xI2C1Done : xI2C2Done;
////	if (settings->address != 0x69 << 1) { printf("try take semphr (write): %d\n", uxSemaphoreGetCount(i2c_semphr)); }
//
//	xSemaphoreTake(i2c_mutex, MAX_I2C_MUTEX_DELAY);
//	xSemaphoreTake(i2c_done, 0);
//
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(
//	  settings->hi2c,
//	  settings->address,
//	  reg, I2C_MEMADD_SIZE_8BIT,
//	  data, size
//	);
//
////	printf("write 0x%X from 0x%X gave %d\n", reg, (settings->address) >> 1, status);
//
//	if (status == HAL_OK && settings->expect_response) {
//		xSemaphoreTake(i2c_done, MAX_I2C_DONE_DELAY);
//	}
//
//	xSemaphoreGive(i2c_mutex);
//
//	return status;
//}

HAL_StatusTypeDef i2c_write_register(i2cSettings *settings, uint8_t reg, uint8_t data) {
	return i2c_write_registers(settings, reg, &data, 1);
}

//HAL_StatusTypeDef i2c_read_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size) {
//	SemaphoreHandle_t i2c_mutex = (settings->hi2c->Instance == I2C1) ? xI2C1Mutex : xI2C2Mutex;
//	SemaphoreHandle_t i2c_done = (settings->hi2c->Instance == I2C1) ? xI2C1Done : xI2C2Done;
////	if (settings->address != 0x69 << 1) { printf("try take semphr (read): %d\n", uxSemaphoreGetCount(i2c_semphr)); }
//
//	xSemaphoreTake(i2c_mutex, MAX_I2C_MUTEX_DELAY);
//	xSemaphoreTake(i2c_done, 0);
//
//	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(
//	  settings->hi2c,
//	  settings->address,
//	  reg, I2C_MEMADD_SIZE_8BIT,
//	  data, size
//	);
//
////	printf("read 0x%X from 0x%X gave %d\n", reg, (settings->address) >> 1, status);
//
//	if (status == HAL_OK && settings->expect_response) {
//		xSemaphoreTake(i2c_done, MAX_I2C_DONE_DELAY);
//	}
//
//	xSemaphoreGive(i2c_mutex);
//
//	return status;
//}

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
