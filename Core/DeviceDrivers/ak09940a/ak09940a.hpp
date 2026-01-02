/*
 * ak09940a.h
 *
 *  Created on: Jul 25, 2025
 *      Author: tomwolcott
 */

#ifndef AK09940A_AK09940A_H_
#define AK09940A_AK09940A_H_

#include "main.h"
#include <stdbool.h>
#include "linmath.h"
#include "i2c.hpp"
#include <vector>

extern DeviceI2C AK09940A_I2C;

#define AK09940A_I2C_PORT hi2c2
#define AK09940A_I2C_ADDRESS (0x0F << 1)

// https://www.mouser.cn/datasheet/2/1431/ak09940a_en_datasheet_myakm-3244294.pdf#page=13
typedef enum {
	AK09940A_PowerDown = 0x00,
	AK09940A_SingleMeasurement = 0x01, // Transitions back to power down automatically
	AK09940A_Continuous_10Hz = 0x02,
	AK09940A_Continuous_20Hz = 0x04,
	AK09940A_Continuous_50Hz = 0x06,
	AK09940A_Continuous_100Hz = 0x08,
	AK09940A_Continuous_200Hz = 0x0A,
	AK09940A_Continuous_400Hz = 0x0C,
	AK09940A_Continuous_1000Hz = 0x0E,
	AK09940A_Continuous_2500Hz = 0x0F,
	AK09940A_ExternalTrigger = 0x18,
	AK09940A_SelfTest = 0x10, // Transitions back to power down automatically
} AK09940A_OperationMode;

// https://www.mouser.cn/datasheet/2/1431/ak09940a_en_datasheet_myakm-3244294.pdf#page=40
// https://www.mouser.cn/datasheet/2/1431/ak09940a_en_datasheet_myakm-3244294.pdf#page=41
typedef enum {
	AK09940A_LowPowerDrive1 = 0x00,
	AK09940A_LowPowerDrive2 = 0x01,
	AK09940A_LowNoiseDrive1 = 0x02,
	AK09940A_LowNoiseDrive2 = 0x03,
	AK09940A_UltraLowNoiseDrive = 0x04 // MT0, MT1 don't seem to matter when MT2=1 is set for ultra-low
}  AK09940A_DriveMode;

struct AK09940A_Output {
	int32_t mag[3];
	float temp;
	bool data_overrun;

	void into_message(std::vector<uint8_t> &data);
};

class AK09940A_Dev {
private:
	AK09940A_OperationMode operation_mode;
	AK09940A_DriveMode drive_mode;
	int32_t mag_bias[3];
    float mag_scale[3];
	bool temp_on;
	bool fifo_on;

	int set_power_down();
	int read_measurement(AK09940A_Output *output, uint32_t timeout_ms);

public:
	int update_offset_registers();

	void init(AK09940A_OperationMode operation_mode, AK09940A_DriveMode drive_mode);

	AK09940A_Dev();

	void set_control_registers();

	int get_avg_measurement(int num_avgs, AK09940A_Output *output);

	bool big_magnet_nearby();

	AK09940A_Output single_measure();

	int print_self_test(char *s);

	void update_calibration(const int32_t new_bias[3], const float new_scale[3]) {
	    for (int i = 0; i < 3; i++) {
	        mag_bias[i] = new_bias[i];
	        mag_scale[i] = new_scale[i];
	    }
	}
};

#endif /* AK09940A_AK09940A_H_ */
