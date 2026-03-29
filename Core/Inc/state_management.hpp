/*
 * state_managment.hpp
 *
 *  Created on: Oct 31, 2025
 *      Author: tomwolcott
 */

#ifndef INC_STATE_MANAGEMENT_HPP_
#define INC_STATE_MANAGEMENT_HPP_

#include "cpp_main.hpp"
#include "cmsis_os.h"
#include "cpp_freertos_helpers.hpp"
#include <functional>

#include "sml.hpp"
namespace sml = boost::sml;
/*
#define TASK(func, attributes, params) { \
        func, \
        (osThreadAttr_t){ \
            .name = #func, \
            .attr_bits = (attributes).attr_bits, \
            .cb_mem = (attributes).cb_mem, \
            .cb_size = (attributes).cb_size, \
            .stack_mem = (attributes).stack_mem, \
            .stack_size = (attributes).stack_size, \
            .priority = (attributes).priority, \
            .tz_module = (attributes).tz_module, \
            .reserved = (attributes).reserved, \
        }, \
        params \
    }
*/
struct Task {
	osThreadFunc_t task;
	osThreadAttr_t attributes;
	void* parameters;
	osThreadId_t handle = nullptr;
	bool is_task_dead = false;
	bool needs_termination = false;
	uint32_t free_stack_space = 0xffffffff;

	Task(osThreadFunc_t task, osThreadAttr_t attributes, void* parameters) : task(task), attributes(attributes), parameters(parameters) {
	}

	Task(osThreadFunc_t task, osThreadAttr_t attributes, void* parameters, bool needs_termination) : task(task), attributes(attributes), parameters(parameters), needs_termination(needs_termination) {
	}

	bool spawn() {
		is_task_dead = false;

		handle = osThreadNew(task, this, &(attributes));

    	return (handle != NULL);
	}

	bool despawn() {
		is_task_dead = true;

		if (needs_termination) {
			if (handle != nullptr) {
				osThreadTerminate(handle);
			}
		}

    	handle = nullptr;
    	return true;
	}

	uint32_t measure_free_stack() {
		free_stack_space = uxTaskGetStackHighWaterMark( NULL );

		return free_stack_space;
	}
};

template<size_t N, Task state_tasks[N]>
void enterStateAction() {
    for (size_t i = 0; i < N; ++i) {
    	if (state_tasks[i].task)
    		state_tasks[i].spawn();
    }
}


template<size_t N>
auto getEnterStateAction(Task (&state_tasks)[N]) {
    return [state_tasks]() mutable {
        for (size_t i = 0; i < N; ++i) {
        	if (state_tasks[i].task)
        		state_tasks[i].spawn();
        }
    };
}

template<size_t N, Task state_tasks[N]>
void exitStateAction() {
    for (size_t i = 0; i < N; ++i) {
    	state_tasks[i].despawn();
    }
}

template<size_t N>
auto getExitStateAction(Task (&state_tasks)[N]) {
    return [state_tasks]() mutable {
        for (size_t i = 0; i < N; ++i) {
        	state_tasks[i].despawn();
        }
    };
}

const osThreadAttr_t defaultTask_attributes = {
  .name = "NEVER",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal
};

namespace SetupMode {
	using namespace sml;

	// Unconnected Tasks
	void searchingForConnection(void* parameters);
	void unconnectedBlinkBlonk(void* parameters);
	void respondToInput(void *parameters);

	static Task UNCONNECTED_TASKS[] = {
		Task(searchingForConnection, {.name = "connSearch", .stack_size = 256, .priority = (osPriority_t) osPriorityNormal}, nullptr),
		Task(unconnectedBlinkBlonk, {.name = "blinkblonk", .stack_size = 256, .priority = (osPriority_t) osPriorityNormal}, nullptr),
		Task(respondToInput, {.name = "inputResp", .stack_size = 1500, .priority = (osPriority_t) osPriorityNormal}, nullptr)
	};

	// Connected Tasks
	void collectData(void *parameters);
	void sendData(void *parameters);
	void calibrationRoutine(void *parameters);
	void debugPrinter(void *parameters);
	void handleActionQueue(void *parameters);
	void depthAndSpeedControl(void *parameters);

	namespace ConnectedMode {
		static Task CALIBRATING_TASKS[] = {
			Task(calibrationRoutine, {.name = "calibration", .stack_size = 1024, .priority = (osPriority_t) osPriorityHigh}, nullptr),
		};

		static Task SENDING_DATA_TASKS[] = {
			Task(collectData, {.name = "collectData", .stack_size = 1024, .priority = (osPriority_t) osPriorityNormal }, nullptr),
			Task(sendData, {.name = "sendData", .stack_size = 1024, .priority = (osPriority_t) osPriorityNormal}, nullptr),
			Task(respondToInput, {.name = "inputResp_conn", .stack_size = 1500, .priority = (osPriority_t) osPriorityNormal}, nullptr),
			Task(debugPrinter, {.name = "debugPrinter", .stack_size = 600, .priority = (osPriority_t) osPriorityNormal}, nullptr),
			Task(handleActionQueue, {.name = "handleAQ", .stack_size = 1500, .priority = (osPriority_t) osPriorityNormal}, nullptr),
			Task(depthAndSpeedControl, {.name = "depthAndSpeed", .stack_size = 1024, .priority = (osPriority_t) osPriorityNormal}, nullptr)
		};

		// Events
		struct CalibrationStart {};
		struct CalibrationStop {};

		// States
		class Calibrating {};
		class SendingData {};


		// State Machine
		struct SM {
			auto operator()() const {
				return make_transition_table(
					state<Calibrating> <= *state<SendingData> + event<CalibrationStart>,
						                   state<SendingData> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<6, SENDING_DATA_TASKS>),
						                   state<SendingData> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<6, SENDING_DATA_TASKS>),

					state<SendingData> <= state<Calibrating> + event<CalibrationStop>,
						                  state<Calibrating> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<1, CALIBRATING_TASKS>),
						                  state<Calibrating> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<1, CALIBRATING_TASKS>)
				);
			}
		};
	}

	// Events
	struct EnterConnected {};
	struct EnterUnconnected {};

	// States
	class Unconnected {};
	using Connected = ConnectedMode::SM;

	// State Machine
	struct SM {
		auto operator()() const {
			return make_transition_table(
				state<Connected>   <= *state<Unconnected> + event<EnterConnected>,
				                      state<Unconnected> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<3, UNCONNECTED_TASKS>),
				                      state<Unconnected> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<3, UNCONNECTED_TASKS>),

				state<Unconnected> <= state<Connected> + event<EnterUnconnected>
			);
		}
	};
}

namespace SystemModes {
	using namespace sml;

	// Setup Tasks
	void repeatEchoes(void* parameters);
	void watchout(void* parameters);
	void motor_current_control(void* parameters);

	static Task SETUP_TASKS[] = {
		Task(repeatEchoes, {.name = "echoReply", .stack_size = 400, .priority = (osPriority_t) osPriorityNormal}, nullptr),
		Task(watchout, {.name = "watchout", .stack_size = 1000, .priority = (osPriority_t) osPriorityHigh}, nullptr),
		Task(motor_current_control, {.name = "motorLimiter", .stack_size = 1500, .priority = (osPriority_t) osPriorityHigh}, nullptr)
	};

	// Events
	struct EnterActive {};
	struct EnterSleep {};
	struct ReEnterSetup {};
	struct StartStateMachine {};

	// Main States
    const auto idle = state<class idle>;
	using Setup = SetupMode::SM;
	class Active {};
	class Sleep {};

	// State Machine
	struct SM {
		auto operator()() const {
			return make_transition_table(
				// toState <= fromState + event [guard] / action:
				// On `Event` if `guard` is true perform `action` and transition from `fromState` to `toState`
				state<Setup>  <= *idle + event<StartStateMachine>,
					state<Setup> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<3, SETUP_TASKS>),
					state<Setup> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<3, SETUP_TASKS>),

				state<Active> <= state<Setup> + event<EnterActive>,

				state<Sleep>  <= state<Active> + event<EnterSleep>,
				state<Sleep>  <= state<Setup> + event<EnterSleep>,

				state<Setup>  <= state<Active> + event<ReEnterSetup>
			);
		}
	};
}

extern MutexLazy<sml::sm<SystemModes::SM>> systemModesSM;

#include "AdcData.hpp"
#include "AK09940A.hpp"
#include "ICM42688.hpp"
#include "localization.hpp"
#include "MotorControl.hpp"
#include "control_loops.hpp"
#include "Message.hpp"
#include "ms5837.h"

struct Data {
	AdcData adcData;
	AK09940A_Output ak09940a_output;
	AK09940A_Dev ak09940a_dev;
	ICM42688_Data icm42688_output;
	ICM42688 icm42688_dev;
	ComplementaryFilter localization;
	LocalizationOutput localization_output;
};

struct OuterData {
	ms5837_output_t ms5837_output;
	uint32_t lastTimeSurfaced;
	uint32_t lastTimeDived;
};

enum ControlLoopState {
	CL_STATE_OFF = 0,
	CL_STATE_VERTICAL_ROLL = 1,
	CL_STATE_HORIZONTAL_ROLL = 2,
	CL_STATE_PITCH = 3,
	CL_STATE_YAW = 4,
	CL_STATE_ALL_ORI = 5
};

struct ControlTargets {
	float roll;
	float pitch;
	float yaw;
};

struct PIDs {
	RollCL roll = RollCL();
	OrientationCL oriCL = OrientationCL();
	ActionItemMovingMode state = AQ_ITEM_MM_NONE;
	ControlTargets targets = {0.0, 0.0, 0.0};
	quat<float> target_ori = identity_quat<float>();
	float speed = 0.0;
};

struct ActionQueueState {
	ActionQueue aq;
	int index;
	bool changed = false;

	ActionQueueState() { aq = ActionQueue(); index = 0; changed = false; }
	ActionQueueState(ActionQueue aq) : aq(aq) { index = 0; changed = true; }
	void shift() {
		index++;
		changed = true;
	}
};

#include "config.hpp"

// Both defined in cpp_main.cpp
extern MutexLazy<Data> dataMutex;
extern MutexLazy<Config> configMutex;
extern MutexLazy<MotorControl> motorControlMutex;
extern MutexLazy<PIDs> pidMutex;
extern MutexLazy<ActionQueueState> actionQueueMutex;
extern MutexLazy<ms5837_t> ms5837Mutex;
extern MutexLazy<OuterData> outerDataMutex;


#endif /* INC_STATE_MANAGEMENT_HPP_ */
