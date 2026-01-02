/*
 * SystemMode.cpp
 *
 *  Created on: Oct 31, 2025
 *      Author: tomwolcott
 */

#include <cstdio>
#include "state_management.hpp"
#include "main.h"
#include <string.h>
#include "Message.hpp"

static uint32_t stack_expense[3] = {0, 0, 0};

namespace SetupMode {
	using namespace SetupMode;

	void __NO_RETURN searchingForConnection(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			stack_expense[0] = 4 * uxTaskGetStackHighWaterMark(NULL);
			Message::pingWithMs().send();
			osDelay(2000);
		}

		osThreadExit();
	}

	void __NO_RETURN unconnectedBlinkBlonk(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			stack_expense[1] = 4*uxTaskGetStackHighWaterMark(NULL);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			osDelay(500);
		}

		osThreadExit();
	}

	void __NO_RETURN respondToInput(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			Message msg = Message::receiveWait();

//			msg.printDataToScreen(0, 0, 4, 6);

			auto sm_lock = systemModesSM.get_lock();

			if (sm_lock->is<decltype(sml::state<SM>)>(sml::state<Unconnected>)) {
				sm_lock->process_event(EnterConnected {});
			}

			sm_lock.unlock();
			
			printf("Received message: %s\n", msg.typeToString().c_str());

			switch (msg.type()) {
			case MESSAGE_TYPE_SEND_CONFIG: {
				auto config_lock = configMutex.get_lock();
				printf("Send config recieved!\n");

				*config_lock = Config::from_message(msg);

//				lock->save_into_flash();

				auto data_lock = dataMutex.get_lock();
				config_lock->update_sensors(&data_lock->ak09940a_dev, &data_lock->icm42688_dev);
				data_lock.unlock();

				config_lock.unlock();
				break;
			} case MESSAGE_TYPE_ACTION: {
				ActionMsg action = msg.as_action();

				switch (action.type()) {
				case ACTION_TYPE_SEND_CONFIG: {
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
					printf("Send config action sent!\n");
					break;
				} case ACTION_TYPE_CALIBRATE: {
					CalibrationActionMsg calibration = action.as_calibration();

					printf("Calibration action sent!\n");
					break;
				} default: {

				}}
				break;
			} default: {

			}}

			stack_expense[2] = 4*uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	static int collectDataCount = 0;

	void __NO_RETURN collectData(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			auto lock = dataMutex.get_lock();
			lock->adcData = AdcData::from_buffer();
			lock->icm42688_output = lock->icm42688_dev.get_data();
			lock->ak09940a_output = lock->ak09940a_dev.single_measure();
			lock.unlock();
			collectDataCount++;
//			osDelay(100);
			stack_expense[0] = 4 * uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	void __NO_RETURN sendData(void *parameters) {
		Task *this_task = (Task *)parameters;
		uint32_t last_t = HAL_GetTick();

		while (!this_task->is_task_dead) {
			uint16_t rate_hz = 1000 * collectDataCount / (HAL_GetTick() - last_t);
			collectDataCount = 0;

			last_t = HAL_GetTick();
			OtherData other_data = OtherData(last_t, rate_hz);

			auto lock = dataMutex.get_lock();
			Message msg = Message::sendData(
					&lock->adcData,
					&lock->ak09940a_output,
					&lock->icm42688_output,
					&other_data
			);
			lock.unlock();

			msg.send();

			osDelay(50);
			stack_expense[1] = 4 * uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	void __NO_RETURN magCalibration(void *parameters) {
		// wait X seconds after unplug OR button press

		// run calibration routine

		// wait for signal from website to finish or continue with routine

		// upload data / calibration results from routine

		// exit back to the sending data mode
	}
}
