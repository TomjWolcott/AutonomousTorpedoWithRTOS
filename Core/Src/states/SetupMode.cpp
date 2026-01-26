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
#include "qvm_lite.hpp"

static uint32_t stack_expense[3] = {0, 0, 0};

namespace SystemModes {
	void __NO_RETURN repeatEchoes(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			Message msg = Message::receiveWait([](Message &msg) {
				return msg.type() == MESSAGE_TYPE_ECHO && msg.asEchoOrigin() == ECHO_ORIGIN_CONTROLLER;
			});

			if (!this_task->is_task_dead) {
				msg.send();
			}
		}

		osThreadExit();
	}

	void __NO_RETURN watchout(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			bool go_to_sleep = false;

			auto motor_lock = motorControlMutex.get_lock();

//			MotorStats stats[4] = {
//				motor_lock->get_motor_stats(M_TL),
//				motor_lock->get_motor_stats(M_TR),
//				motor_lock->get_motor_stats(M_BL),
//				motor_lock->get_motor_stats(M_BR)
//			};

			float batt_v = motor_lock->estimated_true_batt_v();

			motor_lock.unlock();

//			float total_current = (stats[0].current + stats[1].current + stats[2].current + stats[3].current);

//			go_to_sleep |= total_current > 4.0;

			go_to_sleep |= batt_v < 3.6;

			if (go_to_sleep) {
//				printf("GOING TO SLEEP: batt_v: %.3f Volts, total_current: %.3f Amps\n", batt_v, total_current);

				auto sm_lock = systemModesSM.get_lock();
				sm_lock->process_event(EnterSleep {});
				sm_lock.unlock();
			}

			osDelay(10);
		}

		osThreadExit();
	}
}

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
			
			printf("Received message: %s, len: %d\n", msg.typeToString().c_str(), msg.data.size());

			switch (msg.type()) {
			case MESSAGE_TYPE_SEND_CONFIG: {
				auto config_lock = configMutex.get_lock();
				printf("Send config recieved!\n");

				*config_lock = msg.asConfig();
				float x = config_lock->madgwickBeta;

				config_lock->save_into_flash();

//				auto data_lock = dataMutex.get_lock();
//				config_lock->update_sensors(&data_lock->ak09940a_dev, &data_lock->icm42688_dev);
//				data_lock.unlock();

				config_lock.unlock();

				auto data_lock = dataMutex.get_lock();
				data_lock->localization.tuning_parameter = x;
				data_lock.unlock();
				break;
			} case MESSAGE_TYPE_ACTION: {
				ActionMsg action = msg.asAction();

				switch (action.type()) {
				case ACTION_TYPE_SEND_CONFIG: {
					printf("SENDING CONFIG NOW!!\n");
					auto config_lock = configMutex.get_lock();
					Message msg = Message::sendConfig(*config_lock);
					config_lock.unlock();

					msg.send();
					printf("SENT!!\n");
					break;
				} case ACTION_TYPE_SET_MOTOR_SPEEDS: {
					MotorSpeeds motor_speeds = action.asMotorSpeeds();
//					printf("setting motor speeds: [%.4f, %.4f, %.4f, %.4f]\n", motor_speeds.speeds[0], motor_speeds.speeds[1], motor_speeds.speeds[2], motor_speeds.speeds[3]);
					auto motor_lock = motorControlMutex.get_lock();
//					printf("In motor lock\n");

					motor_lock->set_motor_speeds(motor_speeds.speeds);

					motor_lock.unlock();
					break;
				} case ACTION_TYPE_CALIBRATION_MSG: {
					printf("STARTING CALIBRATION ROUTINE!\n");
					auto sm_lock = systemModesSM.get_lock();
					sm_lock->process_event(ConnectedMode::CalibrationStart {});
					sm_lock.unlock();

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
			auto data_lock = dataMutex.get_lock();
			auto config_lock = configMutex.get_lock();
			data_lock->adcData = AdcData::from_buffer();
			data_lock->icm42688_output = data_lock->icm42688_dev.get_data_raw();
			data_lock->ak09940a_output = data_lock->ak09940a_dev.single_measure_raw();
			data_lock->localization.update(
				config_lock->calibrated_acc(data_lock->icm42688_output.acc),
				config_lock->calibrated_mag(data_lock->ak09940a_output.mag),
				config_lock->calibrated_gyro(data_lock->icm42688_output.gyro)
			);

			config_lock.unlock();
			data_lock.unlock();
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

			auto motor_lock = motorControlMutex.get_lock();
			AllMotorStats stats = motor_lock->get_all_motor_stats();
			motor_lock.unlock();

			auto data_lock = dataMutex.get_lock();
			data_lock->localization_output = data_lock->localization.output();
			Message msg = Message::sendData(
					data_lock->adcData,
					data_lock->ak09940a_output,
					data_lock->icm42688_output,
					other_data,
					data_lock->localization_output,
//					std::nullopt
					stats
			);
			data_lock.unlock();

			msg.send();

			osDelay(50);
			stack_expense[1] = 4 * uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	void __NO_RETURN debugPrinter(void *parameters) {
		Task *this_task = (Task *)parameters;
//		Instant prevInstant = getInstant();

		while (!this_task->is_task_dead) {
//			auto motor_lock = motorControlMutex.get_lock();
////			AllMotorStats stats = motor_lock->get_all_motor_stats();
//			MotorStats stats = motor_lock->get_motor_stats(M_TL);
//			AdcData data = motor_lock->data;
//			motor_lock.unlock();
//
//			printf("tl current: %.5f A, voltage: %.5f V, power: %.5f W, ipropi_v = %.5f, ipropi_mv = %d\n", stats.current, stats.voltage, stats.power, data.ipropis_v()[3], data.ipropis_mv[3]);

//			printf(
//				"current: [%.3f, %.3f, %.3f, %.3f], voltage: [%.3f, %.3f, %.3f, %.3f]\n",
//				stats.stats[0].current,
//				stats.stats[1].current,
//				stats.stats[2].current,
//				stats.stats[3].current,
//
//				stats.stats[0].voltage,
//				stats.stats[1].voltage,
//				stats.stats[2].voltage,
//				stats.stats[3].voltage
//			);


			osDelay(500);
		}

		osThreadExit();
	}

	#define WAIT_FOR_UNPLUG_MS 1000

	void printCalibRoutine(int i, Message &msg) {
		printf("CALIB @ %d (%s: %s)\n", i, msg.typeToString().c_str(), msg.dataToString().c_str());
	}

	using Vec3 = std::array<float, 3>;

	void __NO_RETURN calibrationRoutine(void *parameters) {
		Message msg;
		std::optional<std::vector<Vec3>> data_opt = std::nullopt;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

		printCalibRoutine(0, msg);

		while (1) {
			msg = Message::receiveWait([](Message &msg) {
				return msg.type() == MESSAGE_TYPE_ACTION && msg.asAction().type() == ACTION_TYPE_CALIBRATION_SETTINGS;
			});

			printCalibRoutine(1, msg);

			CalibrationSettings settings = msg.asAction().asCalibrationSettings();
			bool isUnplugged = settings.startSignal == CALIBRATION_START_SIGNAL_ON_UNPLUG;

			while (1) {
				std::optional<Message> msg_opt = Message::receiveWait(WAIT_FOR_UNPLUG_MS);
				printCalibRoutine(2, msg);

				if (isUnplugged && !msg_opt.has_value() && !isDeviceConnected(DEFAULT_PING_WAIT_MS)) {
					osDelay(pdMS_TO_TICKS(settings.waitMsAfterUnplug));
					break;
				}

				if (!msg_opt.has_value()) { continue; }

				msg = msg_opt.value();

				if (
					msg.type() == MESSAGE_TYPE_ACTION &&
					msg.asAction().type() == ACTION_TYPE_CALIBRATION_MSG &&
					msg.asAction().asCalibrationMsg() == CALIBRATION_MSG_START
				) { break; }

				if (
					msg.type() == MESSAGE_TYPE_ACTION &&
					msg.asAction().type() == ACTION_TYPE_CALIBRATION_SETTINGS
				) {
					settings = msg.asAction().asCalibrationSettings();
				}
			}
			printCalibRoutine(3, msg);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

			uint32_t startTime = HAL_GetTick();
			uint32_t loopStartTime;
			uint32_t waitBetweenMeasurements = 1000 / settings.dataCollectRateHz;

			while (HAL_GetTick() - startTime < settings.dataCollectTimeMs) {
				loopStartTime = HAL_GetTick();

				Vec3 vector;

				auto lock = dataMutex.get_lock();
				switch (settings.type) {
				case CALIBRATION_TYPE_MAG: {
					AK09940A_Output mag_output = lock->ak09940a_dev.single_measure_raw();
					for (int i = 0; i < 3; i++) {
						vector[i] = static_cast<float>(mag_output.mag[i]);
					}
					break;
				} case CALIBRATION_TYPE_ACC: {
					ICM42688_Data icm_data = lock->icm42688_dev.get_data_raw();
					for (int i = 0; i < 3; i++) {
						vector[i] = icm_data.acc[i];
					}
					break;
				} case CALIBRATION_TYPE_GYR: {
					ICM42688_Data icm_data = lock->icm42688_dev.get_data_raw();
					for (int i = 0; i < 3; i++) {
						vector[i] = icm_data.gyro[i];
					}
					break;
				} default: {
					vector = {0.0f, 0.0f, 0.0f};
				}}
				lock.unlock();

				if (isUnplugged) {
					data_opt.value().push_back(vector);
				} else {
					Message::sendCalibrationData(std::span{&vector, 1}, false).send();
				}
//				Message x = Message::sendCalibrationData(std::span{&vector, 1}, false);
//
//				printf("vector: [%.3f, %.3f, %.3f] -- %s\n", vector[0], vector[1], vector[2], x.dataToString().c_str());

				osDelay(waitBetweenMeasurements - (HAL_GetTick() - loopStartTime));
			}

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

			if (isUnplugged) {
				while (!isDeviceConnected(DEFAULT_PING_WAIT_MS)) {}

				uint32_t index = 0;
				std::span<Vec3> data_span = data_opt.value();

				const uint32_t maxVec3PerMessage = 250 / 12;

				while (index + maxVec3PerMessage < data_span.size()) {
					Message::sendCalibrationData(data_span.subspan(index, index + maxVec3PerMessage), false).send();

					index += maxVec3PerMessage;
				}

				Message::sendCalibrationData(data_span.subspan(index, data_span.size()), true).send();
			} else {
				Message::sendCalibrationData(std::span<Vec3>(), true).send();
			}
			printCalibRoutine(4, msg);

			msg = Message::receiveWait([](Message &msg) {
				return msg.type() == MESSAGE_TYPE_ACTION &&
					   msg.asAction().type() == ACTION_TYPE_CALIBRATION_MSG && (
						   msg.asAction().asCalibrationMsg() == CALIBRATION_MSG_DONE ||
						   msg.asAction().asCalibrationMsg() == CALIBRATION_MSG_GO_AGAIN
					   );
			});

			printCalibRoutine(5, msg);

			CalibrationMsgType calibrationMessage = msg.asAction().asCalibrationMsg();

			if (calibrationMessage == CALIBRATION_MSG_DONE) {
				break;
			} else if (calibrationMessage == CALIBRATION_MSG_GO_AGAIN) {
				continue;
			} else {
				printf("DID NOT EXPECT calibrationMessage: %d", calibrationMessage);
			}
		}
		printCalibRoutine(6, msg);

		auto sm_lock = systemModesSM.get_lock();
		sm_lock->process_event(ConnectedMode::CalibrationStop {});
		sm_lock.unlock();

		osThreadExit();
	}
}
