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

//namespace ConnectedMode {
//	using namespace sml;
//
//	// Events
//	struct CalibrationStart {};
//	struct CalibrationStop {};
//
//	// States
//	class Calibrating {};
//	class SendingData {};
//
//	// Calibrating Tasks
//	void
//
//	// State Machine
//	struct SM {
//		auto operator()() const {
//			return make_transition_table(
//				state<Calibrating> <= *state<SendingData> + event<CalibrationStart>,
//					                   state<SendingData> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<3, UNCONNECTED_TASKS>),
//					                   state<SendingData> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<3, UNCONNECTED_TASKS>),
//
//				state<SendingData> <= state<Calibrating> + event<CalibrationStop>,
//					                  state<Calibrating> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<3, CONNECTED_TASKS>),
//					                  state<Calibrating> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<3, CONNECTED_TASKS>)
//			);
//		}
//	};
//}

namespace SetupMode {
	using namespace sml;

	// Events
	struct EnterConnected {};
	struct EnterUnconnected {};

	// States
	class Unconnected {};
	class Connected {};

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

	static Task CONNECTED_TASKS[] = {
		Task(collectData, {.name = "collectData", .stack_size = 1024, .priority = (osPriority_t) osPriorityNormal}, nullptr),
		Task(sendData, {.name = "sendData", .stack_size = 1024, .priority = (osPriority_t) osPriorityNormal}, nullptr),
		Task(respondToInput, {.name = "inputResp_conn", .stack_size = 1500, .priority = (osPriority_t) osPriorityNormal}, nullptr),
	};

	// State Machine
	struct SM {
		auto operator()() const {
			return make_transition_table(
				state<Connected>   <= *state<Unconnected> + event<EnterConnected>,
				                      state<Unconnected> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<3, UNCONNECTED_TASKS>),
				                      state<Unconnected> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<3, UNCONNECTED_TASKS>),

				state<Unconnected> <= state<Connected> + event<EnterUnconnected>,
				                      state<Connected> + sml::on_entry<_> / static_cast<std::function<void(void)>>(enterStateAction<3, CONNECTED_TASKS>),
				                      state<Connected> + sml::on_exit<_> / static_cast<std::function<void(void)>>(exitStateAction<3, CONNECTED_TASKS>)
			);
		}
	};
}

namespace SystemModes {
	using namespace sml;

	// Relevant functions

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

struct Data {
	AdcData adcData;
	AK09940A_Output ak09940a_output;
	AK09940A_Dev ak09940a_dev;
	ICM42688_Data icm42688_output;
	ICM42688 icm42688_dev;
};

#include "config.hpp"

// Both defined in cpp_main.cpp
extern MutexLazy<Data> dataMutex;
extern MutexLazy<Config> configMutex;


#endif /* INC_STATE_MANAGEMENT_HPP_ */
