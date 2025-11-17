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

MutexLazy<sml::sm<SystemModes::SM>> systemModesSM = MutexLazy<sml::sm<SystemModes::SM>>();

extern "C" __NO_RETURN void cppMainTask(void *argument) {
	systemModesSM.init_mutex();

	auto lock = systemModesSM.get_lock();
	lock->process_event(SystemModes::StartStateMachine{});

	lock.unlock();

	osThreadExit();

}
