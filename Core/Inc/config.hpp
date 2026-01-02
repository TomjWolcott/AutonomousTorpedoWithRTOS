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
#include "Message.hpp"

class Config {
private:
	// Sensors
	int32_t magBias[3];
	float magScale[3];

	float accBias[3];
	float accScale[3];
	float gyrBias[3];

	// PID: TODO

	// Localization
	float madgwickBeta;

	// Motors
	float maxDutyCycle;
	float dutyScaler;

public:
	Config() {}

	static Config from_flash();

	static Config from_message(Message &msg);

	void save_into_flash();

	void into_message(std::vector<uint8_t> &msg_data);

	void update_sensors(AK09940A_Dev *ak09940a_dev, ICM42688 *icm42688_dev);
};



#endif /* INC_CONFIG_HPP_ */
