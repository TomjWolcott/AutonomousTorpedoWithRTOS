/*
 * config.hpp
 *
 *  Created on: Dec 27, 2025
 *      Author: tomwolcott
 */

#ifndef INC_CONFIG_HPP_
#define INC_CONFIG_HPP_

#include "main.h"
#include "ak09940a.hpp"
#include "ICM42688.hpp"
#include "qvm_lite.hpp"
#include <bit>
using namespace boost::qvm;

class Config {
private:
	// Date last set
	uint32_t dateSetMs[2];

	// Sensors
	float magBias[3];
	float magScale[3];

	float accBias[3];
	float accScale[3];
	float gyrBias[3];

	// PID: TODO

	// Localization

	// Motors
	float maxDutyCycle;
	float dutyScaler;

public:
	float madgwickBeta;
	Config() {}

	static Config from_flash();

	static Config from_msg_data(std::vector<uint8_t> &msg_data);

	void save_into_flash();

	void into_msg_data(std::vector<uint8_t> &msg_data);

	void update_sensors(AK09940A_Dev *ak09940a_dev, ICM42688 *icm42688_dev);

	vec<float,3> calibrated_acc(float acc[3]);

	vec<float,3> calibrated_gyro(float gyro[3]);

	vec<float,3> calibrated_mag(int32_t mag[3]);
};



#endif /* INC_CONFIG_HPP_ */
