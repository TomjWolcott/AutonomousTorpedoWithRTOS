/*
 * ak09940a.c
 *
 *  Created on: Jul 25, 2025
 *      Author: tomwolcott
 */

#include "ak09940a.hpp"
#include "registers.hpp"
#include <stdio.h>

DeviceI2C AK09940A_I2C = DeviceI2C(&hi2c2, (0x0F << 1));

/// Needs 12 bytes
void AK09940A_Output::into_message(std::vector<uint8_t> &data) {
	for (int i = 0; i < 3; i++) {
		uint32_t data_val = (uint32_t)static_cast<int32_t>(mag[i]);
		data.push_back((uint8_t)(data_val >> 24));
		data.push_back((uint8_t)((data_val >> 16) & 0xFF));
		data.push_back((uint8_t)((data_val >> 8) & 0xFF));
		data.push_back((uint8_t)(data_val & 0xFF));
	}
}

#define AK09940RegControl3 0x0;

int AK09940A_Dev::set_power_down() {
	uint8_t data = AK09940A_I2C.read_register(AK09940A_REG_CNTL3);

//	char s[300];
//	sprintf(s, "cntl3: %X", data);
//	print_out(s);

	data = data & 0xE0;

	return AK09940A_I2C.write_register(AK09940A_REG_CNTL3, data);
}

int AK09940A_Dev::update_offset_registers() {
	uint8_t offset_data[6];

	int status = AK09940A_I2C.read_registers(AK09940A_REG_SXL, offset_data, 6);

	return status;
}

AK09940A_Dev::AK09940A_Dev() {}

void AK09940A_Dev::init(AK09940A_OperationMode operation_mode, AK09940A_DriveMode drive_mode) {
	drive_mode = drive_mode;
	operation_mode = operation_mode;
	fifo_on = false;
	temp_on = true;

	uint8_t data[2];

	AK09940A_I2C.read_registers(AK09940A_REG_WIA1, data, 2);

//	printf("ak09940a wia: [%d, %d]", data[0], data[1]);

	set_control_registers();

    for (int i = 0; i < 3; i++) {
        mag_bias[i] = 0;
        mag_scale[i] = 1.0;
    }
}

// Sets the control registers https://www.mouser.cn/datasheet/2/1431/ak09940a_en_datasheet_myakm-3244294.pdf#page=40
void AK09940A_Dev::set_control_registers() {
	uint8_t data[4];

	set_power_down();

	data[0] = ((drive_mode & 0x04) << 5);
	data[1] = (temp_on << 6);
	data[2] = (fifo_on << 7) + ((drive_mode & 0x03) << 5) + operation_mode;
	data[3] = 0x00;

//	write_registers(AK09940A_REG_CNTL1, &data, 4);
//	uint8_t data2 = read_register(AK09940A_REG_CNTL3);
//
//	char s[300];
//	sprintf(s, "cntl3: %X", data2);
//	print_out(s);

	AK09940A_I2C.write_registers(AK09940A_REG_CNTL1, data, 4);
}

int AK09940A_Dev::get_avg_measurement(int num_avgs, AK09940A_Output *output) {
	int final_status = 0;

	set_power_down();


	output->data_overrun = false;
	for (int j = 0; j < 3; j++)
		output->mag[j] = 0;
	output->temp = 0.0;

	for (int i = 0; i < num_avgs; i++) {
		AK09940A_Output single_output = single_measure();

		output->data_overrun |= single_output.data_overrun;
		for (int j = 0; j < 3; j++)
			output->mag[j] += single_output.mag[j];
		output->temp += single_output.temp / (float)num_avgs;
	}

	for (int j = 0; j < 3; j++)
		output->mag[j] /= num_avgs;

	return final_status;
}

static const int32_t BIG_MAGNET_LIMIT_uT = 500;

bool AK09940A_Dev::big_magnet_nearby() {
	AK09940A_Output output;
	get_avg_measurement(3, &output);

	int32_t mag_uT[3] = {output.mag[0] / 1000, output.mag[1] / 1000, output.mag[2] / 1000};
	int32_t mag_mag_sqr_uT = (mag_uT[0] * mag_uT[0]) + (mag_uT[1] * mag_uT[1]) + (mag_uT[2] * mag_uT[2]);

	return mag_mag_sqr_uT >= BIG_MAGNET_LIMIT_uT * BIG_MAGNET_LIMIT_uT;
}

AK09940A_Output AK09940A_Dev::single_measure() {
	AK09940A_OperationMode prev_op_mode = operation_mode;
	AK09940A_Output output = { 0 };
	operation_mode = AK09940A_SingleMeasurement;
	set_control_registers();

	read_measurement(&output, 100);

	operation_mode = prev_op_mode;

	return output;
}

AK09940A_Output AK09940A_Dev::single_measure_raw() {
	AK09940A_OperationMode prev_op_mode = operation_mode;
	AK09940A_Output output = { 0 };
	operation_mode = AK09940A_SingleMeasurement;
	set_control_registers();

	read_measurement_raw(&output, 100);

	operation_mode = prev_op_mode;

	return output;
}

int AK09940A_Dev::read_measurement_raw(AK09940A_Output *output, uint32_t timeout_ms) {
	uint8_t data_ready = 0;
	uint32_t time = HAL_GetTick();

	while (!data_ready && HAL_GetTick() - time < timeout_ms) {
		AK09940A_I2C.read_registers(AK09940A_REG_ST, &data_ready, 1);

		data_ready &= 1;
	}

	if (!data_ready)
		return 5;

//	read_register(AK09940A_REG_ST1);

	uint8_t data[10];

	int status_meas = AK09940A_I2C.read_registers(AK09940A_REG_HXL, data, 10);

	if (status_meas)
		return 6;

	uint8_t st2_data;
	int status_st2 = AK09940A_I2C.read_registers(AK09940A_REG_ST2, &st2_data, 1);

	if (status_st2)
		return 7;

	for (int i = 0; i < 3; i++) {
		output->mag[i] = 10 * (
			(int32_t)(data[3*i]) +
			256 * (int32_t)(data[3*i+1]) +
			65536 * ((int32_t)(data[3*i+2] & 1) - (int32_t)(data[3*i+2] & 2))
		);
	}

	output->temp = 30.0 - ((float)(data[9] & 0x7F) - (float)(data[9] & 0x80)) / 1.7;
	output->data_overrun = st2_data & 1;

	return 0;
}

int AK09940A_Dev::read_measurement(AK09940A_Output *output, uint32_t timeout_ms) {
	int fn_output = read_measurement_raw(output, timeout_ms);

	for (int i = 0; i < 3; i++) {
	    output->mag[i] = (int32_t)((float)(output->mag[i] - mag_bias[i]) * mag_scale[i]);
	}

	return fn_output;
}

int AK09940A_Dev::print_self_test(char *s) {
	AK09940A_Output output;
	set_power_down();
	drive_mode = AK09940A_LowNoiseDrive2;
	operation_mode = AK09940A_SelfTest;
	set_control_registers();
	int status = read_measurement(&output, 100);

//    sprintf(s, "AK09940A Self Test: (status=%d)\r\n    mag_x: -1200 <= %d <= -300\r\n    mag_y: 300 <= %d <= 1200\r\n    mag_z: -1600 <= %d <= -400\r\n", status, output.mag[0] / 10, output.mag[1] / 10, output.mag[2] / 10);

	return status;
}
