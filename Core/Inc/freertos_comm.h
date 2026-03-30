/*
 * freertos_comm.h
 *
 *  Created on: Mar 29, 2026
 *      Author: tomwolcott
 */

#ifndef INC_FREERTOS_COMM_H_
#define INC_FREERTOS_COMM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define FREERTOS_COMM_I2C1_TX_EVENT (0x01)
#define FREERTOS_COMM_I2C1_RX_EVENT (0x02)
#define FREERTOS_COMM_I2C2_TX_EVENT (0x04)
#define FREERTOS_COMM_I2C2_RX_EVENT (0x08)
#define FREERTOS_COMM_I2C1_EVENT (FREERTOS_COMM_I2C1_TX_EVENT | FREERTOS_COMM_I2C1_RX_EVENT)
#define FREERTOS_COMM_I2C2_EVENT (FREERTOS_COMM_I2C2_TX_EVENT | FREERTOS_COMM_I2C2_RX_EVENT)

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t address;
} i2cSettings;

void i2c_interrupt_handler_tx(I2C_HandleTypeDef *hi2c);
void i2c_interrupt_handler_rx(I2C_HandleTypeDef *hi2c);
void init_freertos_i2c();

HAL_StatusTypeDef i2c_write_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size);
HAL_StatusTypeDef i2c_write_register(i2cSettings *settings, uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2c_read_registers(i2cSettings *settings, uint8_t reg, uint8_t *data, uint16_t size);
uint8_t i2c_read_register(i2cSettings *settings, uint8_t reg);
uint8_t i2c_read_register_with_code(i2cSettings *settings, uint8_t reg, HAL_StatusTypeDef *error_code);

#ifdef __cplusplus
}
#endif

#endif /* INC_FREERTOS_COMM_H_ */
