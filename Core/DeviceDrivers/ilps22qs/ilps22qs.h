/*
 * ilps22qs.h
 *
 *  Created on: Jul 26, 2025
 *      Author: tomwolcott
 */

#ifndef ILPS22QS_ILPS22QS_H_
#define ILPS22QS_ILPS22QS_H_

#include "main.h"
#include "ilps22qs_reg.h"

#define ILPS22QS_I2C_PORT hi2c2
#define ILPS22QS_I2C_ADDRESS (0x5C << 1)

typedef enum {
	ILPS22QS_ONE_SHOT = 0x00,
	ILPS22QS_1_HZ = 0x01,
	ILPS22QS_4_HZ = 0x02,
	ILPS22QS_10_HZ = 0x03,
	ILPS22QS_25_HZ = 0x04,
	ILPS22QS_50_HZ = 0x05,
	ILPS22QS_75_HZ = 0x06,
	ILPS22QS_100_HZ = 0x07,
	ILPS22QS_200_HZ = 0x08,
} ILPS22QS_OutputDataRate;

typedef enum {
	ILPS22QS_AVERAGING_4 = 0x00,
	ILPS22QS_AVERAGING_8 = 0x01,
	ILPS22QS_AVERAGING_16 = 0x02,
	ILPS22QS_AVERAGING_32 = 0x03,
	ILPS22QS_AVERAGING_64 = 0x04,
	ILPS22QS_AVERAGING_128 = 0x05,
	ILPS22QS_AVERAGING_512 = 0x07,
} ILPS22QS_AveragingSelection;

typedef enum {
	ILPS22QS_BOOT_NORMAL = 0x00,
	ILPS22QS_BOOT_REBOOT = 0x01,
} ILPS22QS_Boot;

typedef enum {
	ILPS22QS_FS_1260_hPa = 0x00,
	ILPS22QS_FS_4060_hPa = 0x01,
} ILPS22QS_FsMode;

typedef enum {
	ILPS22QS_LOW_PASS_DISABLED = 0x00, // odr/4, disabled
	ILPS22QS_LOW_PASS_ODR_4 = 0x01, // odr/4, enabled
	ILPS22QS_LOW_PASS_ODR_9 = 0x03, // odr/9, enabled
} ILPS22QS_LowPass;

typedef enum {
	ILPS22QS_NO_RESET = 0x00,
	ILPS22QS_SOFTWARE_RESET = 0x01
} ILPS22QS_SoftwareReset;

typedef enum {
	ILPS22QS_ONE_SHOT_IDLE = 0x00,
	ILPS22QS_ONE_SHOT_NEW_DATASET = 0x01
} ILPS22QS_OneShot;


// TODO allow enabling of AH_QVAR and FIFO
typedef struct {
	ILPS22QS_OutputDataRate output_data_rate;
	ILPS22QS_AveragingSelection averaging_selection;
	ILPS22QS_Boot boot;
	ILPS22QS_FsMode fs_mode;
	ILPS22QS_LowPass low_pass;
	ILPS22QS_SoftwareReset software_reset;
	ILPS22QS_OneShot one_shot;

	float pressure_reference;
} ILPS22QS_Dev;

typedef struct {
	float pressure; // atm
	float temp; // C
} ILPS22QS_Output;

ILPS22QS_Dev ilps22qs_init();

int ilps22qs_set_control(ILPS22QS_Dev *dev);

ILPS22QS_Output ilps22qs_get_data(ILPS22QS_Dev *dev);

#endif /* ILPS22QS_ILPS22QS_H_ */
