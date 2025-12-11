/*
 * veml3328.c
 *
 *  Created on: Aug 9, 2025
 *      Author: tomwolcott
 */

#include "veml3328.h"

static int write_registers(uint8_t reg, uint8_t *pdata, int size) {
	int status = HAL_I2C_Mem_Write(
	  &VEML3328_I2C_PORT,
	  VEML3328_I2C_ADDRESS,
	  reg, 1,
	  pdata, size,
	  HAL_MAX_DELAY
	);

	if (status != HAL_OK) {
		return -1;  // Error
	}
	return 0;  // Success
}

static int write_register(uint8_t reg, uint8_t data) {
	// Write a single byte to the specified register
	return write_registers(reg, &data, 1);
}

static int read_registers(uint8_t reg, uint8_t *pdata, int size) {
	int status = HAL_I2C_Mem_Read(
	  &VEML3328_I2C_PORT,
	  VEML3328_I2C_ADDRESS,
	  reg, I2C_MEMADD_SIZE_8BIT,
	  pdata, size,
	  HAL_MAX_DELAY
	);

	if (status != HAL_OK) {
		return -1;  // Error
	}
	return 0;  // Success
}

static uint8_t read_register(uint8_t reg) {
	uint8_t data;
	read_registers(reg, &data, 1);
	return data;
}

VEML3328_Dev veml3328_init() {
	VEML3328_Dev dev = {
		.sensitivity = VEML3328_HIGH_SENSITIVITY,
		.integration_time = VEML3328_INTEGRATION_TIME_100MS,
		.force_mode = VEML3328_MANUAL_FORCE_MODE,
		.trigger = VEML3328_NO_TRIGGER,
		.shutdown_setting = VEML3328_POWER_ON,
		.shutdown_setting_als = VEML3328_ALL_CHANNELS,
		.dg = VEML3328_DG_x1,
		.gain = VEML3328_GAIN_x1,
	};

	return dev;
}

int veml3328_set_control(VEML3328_Dev *dev) {
	uint8_t control[2] = {0, 0};

	// High
	control[1] |= ((dev->shutdown_setting & 1 > 0) << 15);
	control[1] |= ((dev->shutdown_setting_als & 1 > 0) << 14);
	control[1] |= ((dev->dg & 3) << 12);
	control[1] |= ((dev->gain & 3) << 10);
	control[1] |= ((0 & 3) << 8); // reserved

	// Low
	control[0] |= (0 << 7); // reserved
	control[0] |= ((dev->sensitivity & 1) << 6);
	control[0] |= ((dev->integration_time & 3) << 4);
	control[0] |= ((dev->force_mode & 1) << 3);
	control[0] |= ((dev->trigger & 1) << 2);
	control[0] |= ((0 & 1) << 2); // reserved bit
	control[0] |= ((dev->shutdown_setting & 1 > 0) << 0);

	return write_registers(VEML3328_CONTROL, control, 2);
}

VEML3328_Output veml3328_get_data() {
	VEML3328_Output output = { 0 };
	uint8_t data[10] = { 0 };

	read_registers(VEML3328_DATA_CLEAR, data, 10);

	output.clear = (uint16_t)(data[1]) * 256 + (uint16_t)(data[0]);
	output.red = (uint16_t)(data[3]) * 256 + (uint16_t)(data[2]);
	output.green = (uint16_t)(data[5]) * 256 + (uint16_t)(data[4]);
	output.blue = (uint16_t)(data[7]) * 256 + (uint16_t)(data[6]);
	output.ir = (uint16_t)(data[9]) * 256 + (uint16_t)(data[8]);

	return output;
}





