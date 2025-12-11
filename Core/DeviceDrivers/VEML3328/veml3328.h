/*
 * veml3328.h
 *
 *  Created on: Aug 9, 2025
 *      Author: tomwolcott
 */

#ifndef VEML3328_VEML3328_H_
#define VEML3328_VEML3328_H_

#include "veml3328_reg.h"
#include "main.h"

#define VEML3328_I2C_ADDRESS (0x10 << 2)
#define VEML3328_I2C_PORT hi2c2

typedef enum {
	VEML3328_HIGH_SENSITIVITY,
	VEML3328_LOW_SENSITIVITY,
} VEML3328_Sensitivity;

typedef enum {
	VEML3328_INTEGRATION_TIME_50MS = 0x00,
	VEML3328_INTEGRATION_TIME_100MS = 0x01,
	VEML3328_INTEGRATION_TIME_200MS = 0x02,
	VEML3328_INTEGRATION_TIME_400MS = 0x03,
} VEML3328_IntegrationTime;

typedef enum {
	VEML3328_AUTO_FORCE_MODE = 0x00,
	VEML3328_MANUAL_FORCE_MODE = 0x01,
} VEML3328_ForceMode;

typedef enum {
	VEML3328_NO_TRIGGER = 0x00,
	VEML3328_TRIGGER_SINGLE_MEASUREMENT = 0x01
} VEML3328_Trigger;

typedef enum {
	VEML3328_POWER_ON = 0x00,
	VEML3328_SHUTOFF = 0x11
} VEML3328_ShutdownSetting;

typedef enum {
	VEML3328_ALL_CHANNELS = 0x00,
	VEML3328_G_C_IR_ONLY = 0x01,
} VEML3328_SutdownSettingALS;

typedef enum {
	VEML3328_DG_x1 = 0x00,
	VEML3328_DG_x2 = 0x01,
	VEML3328_DG_x4 = 0x02,
} VEML3328_DG;

typedef enum {
	VEML3328_GAIN_x0_5 = 0x03,
	VEML3328_GAIN_x1 = 0x00,
	VEML3328_GAIN_x2 = 0x01,
	VEML3328_GAIN_x4 = 0x02,
} VEML3328_Gain;

typedef struct {
	VEML3328_Sensitivity sensitivity;
	VEML3328_IntegrationTime integration_time;
	VEML3328_ForceMode force_mode;
	VEML3328_Trigger trigger;
	VEML3328_ShutdownSetting shutdown_setting;
	VEML3328_SutdownSettingALS shutdown_setting_als;
	VEML3328_DG dg;
	VEML3328_Gain gain;
} VEML3328_Dev;

typedef struct {
	uint16_t clear;
	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint16_t ir;
} VEML3328_Output;

VEML3328_Dev veml3328_init();

int veml3328_set_control(VEML3328_Dev *dev);

VEML3328_Output veml3328_get_data();

#endif /* VEML3328_VEML3328_H_ */
