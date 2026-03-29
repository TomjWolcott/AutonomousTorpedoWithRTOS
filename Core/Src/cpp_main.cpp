/*
 * cpp_main.cpp
 *
 *  Created on: Oct 29, 2025
 *      Author: tomwolcott
 */

#include "state_management.hpp"
#include "cpp_main.hpp"
#include "cmsis_os.h"
#include "task.h"
#include "AdcData.hpp"
#include "config.hpp"

MutexLazy<sml::sm<SystemModes::SM>> systemModesSM = MutexLazy<sml::sm<SystemModes::SM>>();
MutexLazy<Data> dataMutex = MutexLazy<Data>();
MutexLazy<Config> configMutex;
MutexLazy<MotorControl> motorControlMutex;
MutexLazy<PIDs> pidMutex;
MutexLazy<ActionQueueState> actionQueueMutex;
MutexLazy<OuterData> outerDataMutex;
MutexLazy<ms5837_t> ms5837Mutex = MutexLazy<ms5837_t>();

static void i2c_scan(I2C_HandleTypeDef *hi2c) {
	uint8_t i, ret;
	for(i=1; i<128; i++) {
	    ret = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i<<1), 3, 5);

	    if (ret == HAL_OK && i < 16) {
	    	printf("0x0%X, ", i);
	    } else if (ret == HAL_OK) {
	    	printf("0x%X, ", i);
	    }

//	    if (ret != HAL_OK) {
//	        printf("    |");
//	    } else if (ret == HAL_OK && i < 16) {
//	        printf("0x0%X|", i);
//	    } else if (ret == HAL_OK) {
//	        printf("0x%X|", i);
//	    }
	}
	printf("\n");
}

extern "C" __NO_RETURN void cppMainTask(void *argument) {
	ssd1306_Init();
	initADC();

	configMutex = MutexLazy<Config>(Config::from_flash());
	configMutex.ensureInitialized();

	MotorControl motor_control = MotorControl();
	motor_control.initialize_pwm();
	motorControlMutex = MutexLazy<MotorControl>(motor_control);

	PIDs pids;
	pids.roll = RollCL(PidParams(0.3, 0.2, 0.1, -1.0, 1.0, -10.0, 10.0));
	pids.oriCL = OrientationCL(PidParams(0.2, 0.0, 0.0, -1.0, 1.0, -10.0, 10.0));
	pidMutex = MutexLazy<PIDs>(pids);

	actionQueueMutex = MutexLazy<ActionQueueState>(ActionQueueState());
	actionQueueMutex.ensureInitialized();

	auto data_lock = dataMutex.get_lock();
	data_lock->ak09940a_dev = AK09940A_Dev();
	data_lock->ak09940a_dev.init(AK09940A_PowerDown, AK09940A_LowNoiseDrive2);
	osDelay(1);
	data_lock->icm42688_dev = ICM42688();
	data_lock->icm42688_dev.begin();
	data_lock->icm42688_dev.setAccelFS(ICM42688::AccelFS::gpm4);
	data_lock->icm42688_dev.setGyroFS(ICM42688::GyroFS::dps62_5);

	data_lock.unlock();

	ms5837Mutex.ensureInitialized();
	auto ms5837_lock = ms5837Mutex.get_lock();
	ms5837_reset( &(*ms5837_lock) );
	osDelay(10);
	ms5837_read_calibration_data( &(*ms5837_lock) );
	ms5837_lock.unlock();

	OuterData outer_data;
	outerDataMutex = MutexLazy<OuterData>(outer_data);
	outerDataMutex.ensureInitialized();

	printf("\nI2C1: \n");
	i2c_scan(&hi2c1);
	printf("\nI2C2: \n");
	i2c_scan(&hi2c2);

//	ssd1306_SetCursor(0, 0);
//	ssd1306_WriteString("2025/2026 Winter", Font_6x8, White);
//	ssd1306_UpdateScreen();

	auto sm_lock = systemModesSM.get_lock();
	sm_lock->process_event(SystemModes::StartStateMachine{});

	sm_lock.unlock();

	osThreadExit();

}
