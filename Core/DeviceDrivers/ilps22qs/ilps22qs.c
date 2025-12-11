/*
 * ilps22qs.c
 *
 *  Created on: Jul 26, 2025
 *      Author: tomwolcott
 */

#include "ilps22qs.h"

static int write_registers(uint8_t reg, uint8_t *pdata, int size) {
	int status = HAL_I2C_Mem_Write(
	  &ILPS22QS_I2C_PORT,
	  ILPS22QS_I2C_ADDRESS,
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
	  &ILPS22QS_I2C_PORT,
	  ILPS22QS_I2C_ADDRESS,
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

ILPS22QS_Dev ilps22qs_init() {
	ILPS22QS_Dev dev = {
		.output_data_rate = ILPS22QS_100_HZ,
		.averaging_selection = ILPS22QS_AVERAGING_512,
		.boot = ILPS22QS_BOOT_NORMAL,
		.fs_mode = ILPS22QS_FS_4060_hPa,
		.low_pass = ILPS22QS_LOW_PASS_ODR_9,
		.software_reset = ILPS22QS_NO_RESET,
		.one_shot = ILPS22QS_ONE_SHOT_IDLE,
	};

	uint8_t rpds[2] = { 0 };

//	read_registers(ILPS22QS_RPDS_L, rpds, 2);

	return dev;
}

int ilps22qs_set_control(ILPS22QS_Dev *dev) {
	uint8_t control[3] = { 0 };

	// reg1
	control[0] |= (dev->output_data_rate & 0x0F) << 3;
	control[0] |= (dev->averaging_selection & 0x07) << 0;

	// reg2
	control[1] |= (dev->boot & 0x01) << 7;
	control[1] |= (dev->fs_mode & 0x01) << 6;
	control[1] |= (dev->low_pass & 0x03) << 4;
	control[1] |= (0 & 0x01) << 3;
	control[1] |= (dev->software_reset & 0x01) << 2;
	control[1] |= (0 & 0x01) << 1;
	control[1] |= (dev->one_shot & 0x01) << 0;

	control[2] = 0;

	return write_registers(ILPS22QS_CTRL_REG1, control, 3);
}

ILPS22QS_Output ilps22qs_get_data(ILPS22QS_Dev *dev) {
	ILPS22QS_Output output = { 0 };
	uint8_t data[5] = { 0 };

	read_registers(ILPS22QS_PRESSURE_OUT_XL, data, 5);

	// I don't understand how pressure could possibly be negative.
	uint32_t pressure = (uint32_t)(data[2]) * 65536 + (uint32_t)(data[1]) * 256 + (uint32_t)(data[0]);
	uint32_t temp = (uint32_t)(data[4]) * 256 + (uint32_t)(data[3]);

	output.pressure = 0.00098692 * (float)(pressure) / ((dev->fs_mode) ? 4096.0 : 2048.0);
	output.temp = (float)(temp) / 100.0;

	return output;
}
