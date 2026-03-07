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
#include "ms5837.h"

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
		char s[100];
		int i = 0;

		while (!this_task->is_task_dead) {
			i = (i + 1) % 30;
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
				xSemaphoreTake(ssd1306_mutex, portMAX_DELAY);
				ssd1306_SetCursor(0, 24);
				sprintf(s, "!!!LOW: %.2fV!!!", batt_v);
				ssd1306_WriteString(s, Font_6x8, White);
				ssd1306_UpdateScreen();
				xSemaphoreGive(ssd1306_mutex);

//				auto sm_lock = systemModesSM.get_lock();
//				sm_lock->process_event(EnterSleep {});
//				sm_lock.unlock();
			} else if (i == 0) {
				xSemaphoreTake(ssd1306_mutex, portMAX_DELAY);
				ssd1306_SetCursor(0, 24);
				sprintf(s, "  Batt: %.2fV   ", batt_v);
				ssd1306_WriteString(s, Font_6x8, White);
				ssd1306_UpdateScreen();
				xSemaphoreGive(ssd1306_mutex);
			}

			osDelay(10);
		}

		osThreadExit();
	}

	void __NO_RETURN motor_current_control(void *parameters) {
		Task *this_task = (Task *)parameters;
		MotorStats stats[4];
		float total_current = 0.0;
		float motorMult = 1.0;

		while (!this_task->is_task_dead) {
			auto motor_lock = motorControlMutex.get_lock();
			motor_lock->global_speed_multiplier = motorMult;
			for (int i = 0; i < 4; i++) {
				stats[i] = motor_lock->get_motor_stats((MotorId)i);
			}
			motor_lock.unlock();

			total_current = 0.0;
			for (int i = 0; i < 4; i++) {
				total_current += stats[i].current;
			}

			if (total_current >= 5.0) {
				motorMult *= 0.995;
				printf("reducing mult to %.2f\n", motorMult);
			} else if (motorMult < 1.0) {
				motorMult = fmin(1.0, motorMult / 0.995);
				printf("increasing mult to %.2f\n", motorMult);
			}

			osDelay(20);
		}

		osThreadExit();
	}
}

namespace ActionQueueTasks {
//	static void __NO_RETURN wait(void *parameters) {
//		uint32_t ms = *((uint32_t *)parameters);
//
//		osDelay(ms);
//
//		osThreadExit();
//	}
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
			printf("Hi!");
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
			} case MESSAGE_TYPE_AQ: {
				auto aq_lock = actionQueueMutex.get_lock();
				*aq_lock = ActionQueue(msg.intoActionQueue());
				aq_lock.unlock();
			} default: {

			}}

			stack_expense[2] = 4*uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	static int collectDataCount = 0;

	// running at ~500Hz
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

			quat<float> ori_quat = data_lock->localization.output().orientation;

			config_lock.unlock();
			data_lock.unlock();

			std::optional<std::array<float, 4>> motor_speeds_frpy = std::nullopt;

			auto pid_lock = pidMutex.get_lock();
			switch (pid_lock->state) {
			case AQ_ITEM_MM_VERTICAL_ROLL_CL_TEST: {
				const vec<float,3> dir = ori_quat * vec<float,3>{0, 0, 1};
				const float target = pid_lock->targets.roll;
				const float measured = atan2(X(dir), Y(dir));

				const float output = pid_lock->roll.update(target, measured);

				motor_speeds_frpy = {0, output, 0, 0};
				break;
			} case AQ_ITEM_MM_MAINTAIN_ORI_TEST: {
				RPYOutputs rpy_outputs = pid_lock->oriCL.update(pid_lock->target_ori, ori_quat);

				motor_speeds_frpy = {0, rpy_outputs.roll, rpy_outputs.pitch, rpy_outputs.yaw};
				break;
			} case AQ_ITEM_MM_FORWARD: {
				RPYOutputs rpy_outputs = pid_lock->oriCL.update(pid_lock->target_ori, ori_quat);

				motor_speeds_frpy = {pid_lock->speed, rpy_outputs.roll, rpy_outputs.pitch, rpy_outputs.yaw};
				break;
			} case AQ_ITEM_MM_NONE: {
				break;
			} default: {

			}}
			pid_lock.unlock();

			if (motor_speeds_frpy.has_value()) {
				auto motor_lock = motorControlMutex.get_lock();
				motor_lock->set_motor_speeds_frpy(motor_speeds_frpy.value());
				motor_lock.unlock();
			}

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
			OtherData other_data = OtherData((1000 * (uint64_t)last_t), rate_hz);

			auto motor_lock = motorControlMutex.get_lock();
			AllMotorStats stats = motor_lock->get_all_motor_stats();
			motor_lock.unlock();

			auto outer_data_lock = outerDataMutex.get_lock();
			ms5837_output_t ms5837_data = outer_data_lock->ms5837_output;
			outer_data_lock.unlock();

			auto data_lock = dataMutex.get_lock();
			data_lock->localization_output = data_lock->localization.output();
			Message msg = Message::sendData(
					data_lock->adcData,
					data_lock->ak09940a_output,
					data_lock->icm42688_output,
					ms5837_data,
					other_data,
					data_lock->localization_output,
					stats
			);
			data_lock.unlock();

			msg.send();

			osDelay(50);
			stack_expense[1] = 4 * uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	#define OUTER_LOOP_REFRESH_RATE (60)

	void __NO_RETURN depthAndSpeedControl(void *parameters){
		Task *this_task = (Task *)parameters;
		uint32_t last_time = HAL_GetTick();
		uint32_t current_time;

		while (!this_task->is_task_dead) {
			auto config_lock = configMutex.get_lock();
			float surface_pressure = config_lock->calibrated_surface_pressure();
			config_lock.unlock();

			auto outer_data_lock = outerDataMutex.get_lock();
			ms5837_output_t ms5837_output = ms5837_get_all_data( &outer_data_lock->ms5837, surface_pressure );
			outer_data_lock->ms5837_output = ms5837_output;
			// filtering on the depth values
//			outer_data_lock->ms5837_output.depth_m = 0.9 * outer_data_lock->ms5837_output.depth_m + 0.1 * ms5837_output.depth_m;
//			outer_data_lock->ms5837_output.temperature_C = 0.9 * outer_data_lock->ms5837_output.temperature_C + 0.1 * ms5837_output.temperature_C;
//			outer_data_lock->ms5837_output.pressure_mbar = 0.9 * outer_data_lock->ms5837_output.pressure_mbar + 0.1 * ms5837_output.pressure_mbar;
			outer_data_lock.unlock();

			current_time = HAL_GetTick();
			osDelay((current_time - last_time >= OUTER_LOOP_REFRESH_RATE) ? 2 : OUTER_LOOP_REFRESH_RATE - (current_time - last_time));
			last_time = current_time;
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
//			auto x = pvPortMalloc(20);
//			printf("I just Malloc'd\n");
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

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			std::optional<Message> msg_opt = Message::sendTaskInfo();

			if (msg_opt.has_value()) {
				msg_opt.value().send();
			}

			osDelay(500);
		}

		osThreadExit();
	}

	const uint32_t OLED_UPDATE_FREQ = 10; // How often the OLED will update in Hz

	void __NO_RETURN handleActionQueue(void *parameters) {
		Task *this_task = (Task *)parameters;
//		int prevIndex = -1;
		char s[100];

		while (!this_task->is_task_dead) {
			auto aq_lock = actionQueueMutex.get_lock();
			int index = aq_lock->index;
			bool changed = aq_lock->changed;
			std::optional<ActionQueueType> item_type_opt = aq_lock->aq.get_nth_type(aq_lock->index);
			aq_lock.unlock();

			if (changed) {
				Message::currentActionNum(index).send();
			}

			if (changed && item_type_opt.has_value()) {
				switch (item_type_opt.value()) {
				case AQ_TYPE_WAIT: {
					auto aq_lock = actionQueueMutex.get_lock();
					AqWait aq_wait = aq_lock->aq.nth_as_wait(aq_lock->index).value();
					aq_lock->shift();
					aq_lock.unlock();

					uint32_t num_updates = (OLED_UPDATE_FREQ * aq_wait.wait_ms) / 1000;
					uint32_t time_left = aq_wait.wait_ms;

					while (time_left > 0) {
						uint32_t dt = (
							num_updates == 0 ||
							(aq_wait.wait_ms / num_updates) == 0
						) ? time_left : std::min(time_left, aq_wait.wait_ms / num_updates);

						if (xSemaphoreTake(ssd1306_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
							ssd1306_SetCursor(0, 0);
							sprintf(s, "Waiting(%dms)       ", (int)time_left);
							ssd1306_WriteString(s, Font_6x8, White);
							ssd1306_UpdateScreen();
							xSemaphoreGive(ssd1306_mutex);
						}

						osDelay(pdMS_TO_TICKS(dt));
						time_left -= dt;
					}

					xSemaphoreTake(ssd1306_mutex, portMAX_DELAY);
					ssd1306_SetCursor(0, 0);
					sprintf(s, "                     ");
					ssd1306_WriteString(s, Font_6x8, White);
					ssd1306_UpdateScreen();
					xSemaphoreGive(ssd1306_mutex);
					break;
				} case AQ_TYPE_WAIT_FOR: {
					auto aq_lock = actionQueueMutex.get_lock();
					ActionItemWaitFor aq_wait_for = aq_lock->aq.nth_as_wait_for(aq_lock->index).value();
					aq_lock->shift();
					aq_lock.unlock();
					int req_condition_mets = 10;

					switch (aq_wait_for) {
					case AQ_ITEM_WAIT_FOR_UPSIDEDOWN: {
						xSemaphoreTake(ssd1306_mutex, portMAX_DELAY);
						ssd1306_SetCursor(0, 0);
						sprintf(s, "Waiting(%d left)", req_condition_mets);
						ssd1306_WriteString(s, Font_6x8, White);
						ssd1306_UpdateScreen();
						xSemaphoreGive(ssd1306_mutex);

						while (req_condition_mets > 0) {
							auto data_lock = dataMutex.get_lock();
							LocalizedAccMag acc_mag = data_lock->localization.output().asLocalizedAccMag();
							data_lock.unlock();

							// Gravity must be pointing in the +z direction (aka the torpedo must be upside down) for req_condition_mets counts
							if (Z(acc_mag.acc) > 0.9) {
								req_condition_mets--;

								if (xSemaphoreTake(ssd1306_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
									ssd1306_SetCursor(0, 0);
									sprintf(s, "Waiting(%d left)     ", req_condition_mets);
									ssd1306_WriteString(s, Font_6x8, White);
									ssd1306_UpdateScreen();
									xSemaphoreGive(ssd1306_mutex);
								}
							}

							osDelay(200);
						}
						break;
					}}

					xSemaphoreTake(ssd1306_mutex, portMAX_DELAY);
					ssd1306_SetCursor(0, 0);
					sprintf(s, "                     ");
					ssd1306_WriteString(s, Font_6x8, White);
					ssd1306_UpdateScreen();
					xSemaphoreGive(ssd1306_mutex);
					break;
				} case AQ_TYPE_SET_MOVING_MODE: {
					auto aq_lock = actionQueueMutex.get_lock();
					ActionItemMovingMode aq_mm = aq_lock->aq.nth_as_moving_mode(aq_lock->index).value();
					aq_lock->shift();
					aq_lock.unlock();

					if (aq_mm == AQ_ITEM_MM_NONE) {
						auto motor_lock = motorControlMutex.get_lock();
						motor_lock->set_motor_speeds_frpy({0, 0, 0, 0});
						motor_lock.unlock();
					}

					auto pid_lock = pidMutex.get_lock();
					pid_lock->state = aq_mm;
					pid_lock.unlock();
					break;
				} case AQ_TYPE_START_RECORDING: {
					auto aq_lock = actionQueueMutex.get_lock();
					AqStartRecording start_recording = aq_lock->aq.nth_as_start_recording(aq_lock->index).value();
					aq_lock->shift();
					aq_lock.unlock();
					break;
				} case AQ_TYPE_STOP_RECORDING: {
					auto aq_lock = actionQueueMutex.get_lock();
					aq_lock->shift();
					aq_lock.unlock();
					break;
				} case AQ_TYPE_SET_TARGETS: {
					auto aq_lock = actionQueueMutex.get_lock();
					AqSetTargets set_targets = aq_lock->aq.nth_as_set_targets(aq_lock->index).value();
					aq_lock->shift();
					aq_lock.unlock();

					auto pid_lock = pidMutex.get_lock();
					pid_lock->target_ori = set_targets.ori;
					pid_lock->speed = set_targets.speed;
					pid_lock.unlock();
					break;
				} default: {

				}}
			} else {
				auto aq_lock = actionQueueMutex.get_lock();
				aq_lock->changed = false;
				aq_lock.unlock();
				osDelay(500);
			}
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
