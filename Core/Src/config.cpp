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
	ak09940a_dev->update_calibration(magBias, magScale);
	icm42688_dev->updateCalibration(accBias, accScale, gyrBias);
}

#define NumFloatsInMsg 38

Config Config::from_message(Message &msg) {
	float floats[NumFloatsInMsg];

	for (int i = 0; i < NumFloatsInMsg; i++) {
		uint32_t data = (
			((uint32_t)(msg.data[4*i + 3 + 6]) << 24) +
			((uint32_t)(msg.data[4*i + 2 + 6]) << 16) +
			((uint32_t)(msg.data[4*i + 1 + 6]) << 8)  +
			 (uint32_t)(msg.data[4*i + 0 + 6])
		);

		floats[i] = *reinterpret_cast<float *>(&data);
	}

	Config config = Config();

	config.magBias[0] = (int32_t)floats[0];
	config.magBias[1] = (int32_t)floats[1];
	config.magBias[2] = (int32_t)floats[2];
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

	// TODO: PID in range floats[15] to floats[34]
	
	config.madgwickBeta = floats[35];
	config.maxDutyCycle = floats[36];
	config.dutyScaler = floats[37];

	return config;
}

void Config::into_message(std::vector<uint8_t> &msg_data) {
	float floats[NumFloatsInMsg] = {0};

	floats[0] = (float)magBias[0];
	floats[1] = (float)magBias[1];
	floats[2] = (float)magBias[2];
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

	for (int i = 0; i < NumFloatsInMsg; i++) {
		uint32_t data = *reinterpret_cast<uint32_t*>(&(floats[i]));
		msg_data.push_back((uint8_t)(data & 0xFF));
		msg_data.push_back((uint8_t)((data >> 8) & 0xFF));
		msg_data.push_back((uint8_t)((data >> 16) & 0xFF));
		msg_data.push_back((uint8_t)(data >> 24));
	}
}
