/*
 * cpp_main.cpp
 *
 *  Created on: Oct 29, 2025
 *      Author: tomwolcott
 */

#include "cpp_main.hpp"
#include "cmsis_os.h"
#include "task.h"
#include <functional>

MutexLazy<std::function<void(sml::sm<SystemModes::SM> *)>> sm_command = MutexLazy<std::function<void(sml::sm<SystemModes::SM> *)>>();

void sendStateMachineCommand(std::function<void(sml::sm<SystemModes::SM> *)> command) {
	auto lock = sm_command.get_lock();

	*lock = command;
}

extern "C" void handleStateMachineCommands(void *parameters) {
	sm_command.init_mutex();
	sml::sm<SystemModes::SM> sm;

	for (;;) {

		auto lock = sm_command.get_lock();
		(*lock)(&sm);
	}
}

extern "C" __NO_RETURN void cppMainTask(void *argument) {

	int used_stack_size = uxTaskGetStackHighWaterMark(NULL);
	osThreadExit();

}
