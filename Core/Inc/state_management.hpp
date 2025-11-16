/*
 * state_managment.hpp
 *
 *  Created on: Oct 31, 2025
 *      Author: tomwolcott
 */

#ifndef INC_STATE_MANAGEMENT_HPP_
#define INC_STATE_MANAGEMENT_HPP_

#include "cmsis_os.h"
#include "cpp_freertos_helpers.hpp"

#include "sml.hpp"
namespace sml = boost::sml;

struct Task {
	osThreadFunc_t task;
	osThreadAttr_t attributes;
	void* parameters;
	osThreadId_t handle = nullptr;
	volatile bool *kill_task_marker = nullptr;

	Task(osThreadFunc_t task, const char *const task_name, osThreadAttr_t input_attributes, void* parameters) : task(task), parameters(parameters) {
		input_attributes.name = task_name;
		attributes = input_attributes;
	}

	bool spawn() {
		if (kill_task_marker != nullptr)
			*kill_task_marker = true;

		handle = osThreadNew(task, parameters, &(attributes));

    	return (handle != NULL);
	}

	bool despawn() {
		if (kill_task_marker != nullptr) {
			*kill_task_marker = true;
		} else if (handle) {
			osThreadTerminate(handle);
		} else {
			return false;
		}

    	handle = nullptr;
    	return true;
	}
};

template<size_t N>
auto getEnterStateAction(Task (&state_tasks)[N]) {
    return [state_tasks]() mutable {
        for (size_t i = 0; i < N; ++i) {
        	if (state_tasks[i].task)
        		state_tasks[i].spawn();
        }
    };
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
		Task(searchingForConnection, "connSearch", defaultTask_attributes, nullptr),
		Task(unconnectedBlinkBlonk, "blinkBlonk", defaultTask_attributes, nullptr),
		Task(respondToInput, "inResp", defaultTask_attributes, nullptr)
	};

	// Connected Tasks
	static Task CONNECTED_TASKS[] = {
		Task(respondToInput, "respToIn", defaultTask_attributes, nullptr)
	};

	// State Machine
	struct SM {
		auto operator()() const {
			return make_transition_table(
				state<Connected>   <= *state<Unconnected> + event<EnterConnected>,
				                      state<Unconnected> + sml::on_entry<_> / getEnterStateAction(UNCONNECTED_TASKS),
				                      state<Unconnected> + sml::on_exit<_> / getExitStateAction(UNCONNECTED_TASKS),

				state<Unconnected> <= state<Connected> + event<EnterUnconnected>,
				                      state<Connected> + sml::on_entry<_> / getEnterStateAction(CONNECTED_TASKS),
				                      state<Connected> + sml::on_exit<_> / getExitStateAction(CONNECTED_TASKS)
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

#endif /* INC_STATE_MANAGEMENT_HPP_ */
