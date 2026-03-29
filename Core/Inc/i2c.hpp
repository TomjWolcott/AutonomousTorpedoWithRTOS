/*
 * i2c.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: tomwolcott
 */

#ifndef INC_I2C_HPP_
#define INC_I2C_HPP_

#include "main.h"

class DeviceI2C {
private:
	I2C_HandleTypeDef *port;
	uint8_t address;
	uint32_t timeout = HAL_MAX_DELAY;
public:
	DeviceI2C(I2C_HandleTypeDef *port, uint8_t address) : port(port), address(address) {}

	HAL_StatusTypeDef write_registers(uint8_t reg, uint8_t *data, uint16_t size);

	HAL_StatusTypeDef write_register(uint8_t reg, uint8_t data);

	HAL_StatusTypeDef read_registers(uint8_t reg, uint8_t *data, uint16_t size);

	uint8_t read_register(uint8_t reg);

	uint8_t read_register(uint8_t reg, HAL_StatusTypeDef *error_code);
};

#endif /* INC_I2C_HPP_ */
