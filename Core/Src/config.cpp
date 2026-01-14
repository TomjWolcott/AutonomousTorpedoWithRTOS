/*
 * config.cpp
 *
 *  Created on: Dec 27, 2025
 *      Author: tomwolcott
 */

#include "config.hpp"
#include "ee.h"

Config config_flash_storage;

Config Config::from_flash() {
	ee_init(&config_flash_storage, sizeof(config_flash_storage));
	ee_read();

	return config_flash_storage;
}

void Config::save_into_flash() {
	ee_init(&config_flash_storage, sizeof(config_flash_storage));
	config_flash_storage = *this;
	ee_write();
}

void Config::update_sensors(AK09940A_Dev *ak09940a_dev, ICM42688 *icm42688_dev) {
	int32_t magBiasInt[3] = { static_cast<int32_t>(magBias[0]), static_cast<int32_t>(magBias[1]), static_cast<int32_t>(magBias[2]) };

	ak09940a_dev->update_calibration(magBiasInt, magScale);
	icm42688_dev->updateCalibration(accBias, accScale, gyrBias);
}

#define NumFloatsInMsg 38

#include <cstdio>

Config Config::from_msg_data(std::vector<uint8_t> &data) {
	float floats[NumFloatsInMsg];

	for (int i = 0; i < NumFloatsInMsg; i++) {
		uint32_t uint32_data = (
			((uint32_t)(data[4*i + 0 + 14]) << 24) +
			((uint32_t)(data[4*i + 1 + 14]) << 16) +
			((uint32_t)(data[4*i + 2 + 14]) << 8)  +
			 (uint32_t)(data[4*i + 3 + 14])
		);

		floats[i] = std::bit_cast<float>(uint32_data);
	}

	Config config = Config();

	config.dateSetMs[0] = ((uint64_t)(data[6]) << 24) | ((uint64_t)(data[7]) << 16) |
	                   ((uint64_t)(data[8]) << 8) | ((uint64_t)(data[9]) << 0);
	config.dateSetMs[1] = ((uint64_t)(data[10]) << 24) | ((uint64_t)(data[11]) << 16) |
	                   ((uint64_t)(data[12]) << 8)  | ((uint64_t)(data[12]) << 0);


	config.magBias[0] = floats[0];
	config.magBias[1] = floats[1];
	config.magBias[2] = floats[2];
	config.magScale[0] = floats[3];
	config.magScale[1] = floats[4];
	config.magScale[2] = floats[5];
	config.accBias[0] = floats[6];
	config.accBias[1] = floats[7];
	config.accBias[2] = floats[8];
	config.accScale[0] = floats[9];
	config.accScale[1] = floats[10];
	config.accScale[2] = floats[11];
	config.gyrBias[0] = floats[12];
	config.gyrBias[1] = floats[13];
	config.gyrBias[2] = floats[14];

	printf("from msg data: { magBias[0]: %.3f, ... , accBias[1]: %.3f, ... }", config.magBias[0], config.accBias[1]);

	// TODO: PID in range floats[15] to floats[34]
	
	config.madgwickBeta = floats[35];
	config.maxDutyCycle = floats[36];
	config.dutyScaler = floats[37];

	return config;
}

void Config::into_msg_data(std::vector<uint8_t> &msg_data) {
	float floats[NumFloatsInMsg] = {0};

	floats[0] = magBias[0];
	floats[1] = magBias[1];
	floats[2] = magBias[2];
	floats[3] = magScale[0];
	floats[4] = magScale[1];
	floats[5] = magScale[2];
	floats[6] = accBias[0];
	floats[7] = accBias[1];
	floats[8] = accBias[2];
	floats[9] = accScale[0];
	floats[10] = accScale[1];
	floats[11] = accScale[2];
	floats[12] = gyrBias[0];
	floats[13] = gyrBias[1];
	floats[14] = gyrBias[2];

	// TODO: PID in range floats[15] to floats[34], filled with zeros for now

	floats[35] = madgwickBeta;
	floats[36] = maxDutyCycle;
	floats[37] = dutyScaler;
	
	msg_data.push_back((uint8_t)(dateSetMs[0] >> 24));
	msg_data.push_back((uint8_t)((dateSetMs[0] >> 16) & 0xFF));
	msg_data.push_back((uint8_t)((dateSetMs[0] >> 8) & 0xFF));
	msg_data.push_back((uint8_t)(dateSetMs[0] & 0xFF));
	msg_data.push_back((uint8_t)(dateSetMs[1] >> 24));
	msg_data.push_back((uint8_t)((dateSetMs[1] >> 16) & 0xFF));
	msg_data.push_back((uint8_t)((dateSetMs[1] >> 8) & 0xFF));
	msg_data.push_back((uint8_t)(dateSetMs[1] & 0xFF));
	printf("sending date bytes: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X] from [%u, %u]\n",
		msg_data[6], msg_data[7], msg_data[8], msg_data[9],
		msg_data[10], msg_data[11], msg_data[12], msg_data[13],
		dateSetMs[0], dateSetMs[1]
	);

	for (int i = 0; i < NumFloatsInMsg; i++) {
		uint32_t data = std::bit_cast<uint32_t>(floats[i]);
		msg_data.push_back((uint8_t)(data >> 24));
		msg_data.push_back((uint8_t)((data >> 16) & 0xFF));
		msg_data.push_back((uint8_t)((data >> 8) & 0xFF));
		msg_data.push_back((uint8_t)(data & 0xFF));
	}
}

// Also converts the axes
vec<float,3> Config::calibrated_acc(float acc[3]) {
	vec<float,3> calibrated = {
		(acc[1] - accBias[1]) / accScale[1],
		-(acc[0] - accBias[0]) / accScale[0],
		-(acc[2] - accBias[2]) / accScale[2]
	};

	return calibrated;
}

vec<float,3> Config::calibrated_gyro(float gyro[3]) {
	vec<float,3> calibrated = {
		(gyro[1] - gyrBias[1]),
		-(gyro[0] - gyrBias[0]),
		-(gyro[2] - gyrBias[2])
	};

	return calibrated;
}

vec<float,3> Config::calibrated_mag(int32_t mag[3]) {
	vec<float,3> calibrated = {
		(static_cast<float>(mag[0]) - magBias[0]) / magScale[0],
		(static_cast<float>(mag[1]) - magBias[1]) / magScale[1],
		(static_cast<float>(mag[2]) - magBias[2]) / magScale[2]
	};

	return calibrated;
}
