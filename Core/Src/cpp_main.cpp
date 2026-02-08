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

MutexLazy<State> stateMutex;
//MutexLazy<PIDs> pidMutex;
uint32_t initialHeapSize;

extern "C" __NO_RETURN void cppMainTask(void *argument) {
	initialHeapSize = xPortGetFreeHeapSize();

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("2025/2026 Winter", Font_6x8, White);
//	ssd1306_SetCursor(0, 8);
//	char s[100];
//	sprintf(s, "heap: %d", initialHeapSize);
//	ssd1306_WriteString(s, Font_6x8, White);
	ssd1306_UpdateScreen();

	osDelay(1000);

	initADC();

	State state = State();

	state.data.ak09940a_dev = AK09940A_Dev();
	state.data.ak09940a_dev.init(AK09940A_PowerDown, AK09940A_LowNoiseDrive2);
	osDelay(1);
	state.data.icm42688_dev = ICM42688();
	state.data.icm42688_dev.begin();
	state.data.icm42688_dev.setAccelFS(ICM42688::AccelFS::gpm4);
	state.data.icm42688_dev.setGyroFS(ICM42688::GyroFS::dps62_5);

	state.config = Config::from_flash();

	state.motor_control = MotorControl();
	state.motor_control.initialize_pwm();

	stateMutex = MutexLazy(state);
	stateMutex.ensureInitialized();

//	auto lock = stateMutex.get_lock();
//	lock->modes.process_event(SystemModes::StartStateMachine{});
//	lock.unlock();

	osThreadExit();

}
