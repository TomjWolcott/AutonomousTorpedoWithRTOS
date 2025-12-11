/*
 * i2c.cpp
 *
 *  Created on: Nov 20, 2025
 *      Author: tomwolcott
 */

#include "i2c.hpp"


HAL_StatusTypeDef DeviceI2C::write_registers(uint8_t reg, uint8_t *data, uint16_t size) {
	return HAL_I2C_Mem_Write(
	  port,
	  address,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  data, size,
	  timeout
	);
}

HAL_StatusTypeDef DeviceI2C::write_register(uint8_t reg, uint8_t data) {
	return write_registers(reg, &data, 1);
}

HAL_StatusTypeDef DeviceI2C::read_registers(uint8_t reg, uint8_t *data, uint16_t size) {
	return HAL_I2C_Mem_Read(
	  port,
	  address,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  data, size,
	  timeout
	);
}

uint8_t DeviceI2C::read_register(uint8_t reg) {
	uint8_t data;
	read_registers(reg, &data, 1);
	return data;
}

uint8_t DeviceI2C::read_register(uint8_t reg, HAL_StatusTypeDef *error_code) {
	uint8_t data;
	*error_code = read_registers(reg, &data, 1);
	return data;
}
