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

extern "C" __NO_RETURN void cppMainTask(void *argument) {
	initADC();

	configMutex = MutexLazy<Config>(Config::from_flash());
	configMutex.ensureInitialized();

	auto data_lock = dataMutex.get_lock();
	data_lock->ak09940a_dev = AK09940A_Dev();
	data_lock->ak09940a_dev.init(AK09940A_PowerDown, AK09940A_LowNoiseDrive2);
	osDelay(1);
	data_lock->icm42688_dev = ICM42688();
	data_lock->icm42688_dev.begin();
	data_lock->icm42688_dev.setAccelFS(ICM42688::AccelFS::gpm4);
	data_lock->icm42688_dev.setGyroFS(ICM42688::GyroFS::dps62_5);

	data_lock.unlock();

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("2025/2026 Winter", Font_6x8, White);
	ssd1306_UpdateScreen();

	auto sm_lock = systemModesSM.get_lock();
	sm_lock->process_event(SystemModes::StartStateMachine{});

	sm_lock.unlock();

	osThreadExit();

}
