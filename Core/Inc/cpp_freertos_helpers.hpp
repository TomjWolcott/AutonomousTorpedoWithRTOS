/*
 * cpp_freertos_helpers.hpp
 *
 *  Created on: Nov 11, 2025
 *      Author: tomwolcott
 */

#ifndef INC_CPP_FREERTOS_HELPERS_HPP_
#define INC_CPP_FREERTOS_HELPERS_HPP_

#include "freertos.h"
#include "semphr.h"

/// Delete the lock after you're done with it or else it will survive until the end of the block
template<typename T>
class MutexLock {
	T *t;
	SemaphoreHandle_t &semaphore;

public:
	MutexLock(TickType_t mutex_wait, T *t_other, SemaphoreHandle_t &semaphore_other) : t(t_other), semaphore(semaphore_other) {
		xSemaphoreTake(semaphore, mutex_wait);
	}

	T *operator->() {
		return t;
	}

	T &operator*() {
		return *t;
	}

	~MutexLock() {
		xSemaphoreGive(semaphore);
	}
};

template<typename T>
class Mutex {
	T t;
	SemaphoreHandle_t semaphore = nullptr;
	TickType_t mutex_wait = portMAX_DELAY;

public:
	Mutex() {
		semaphore = xSemaphoreCreateMutex();
	}

	Mutex(T t_other) {
		t = t_other;
		semaphore = xSemaphoreCreateMutex();
	}

	MutexLock<T> get_lock() {
		return MutexLock<T>(mutex_wait, &t, &semaphore);
	}
};

template<typename T>
class MutexLazy {
	T t;
	SemaphoreHandle_t semaphore = nullptr;
	TickType_t mutex_wait = portMAX_DELAY;

public:
	MutexLazy() {}

	MutexLazy(T t_other) {
		t = t_other;
	}

	void init_mutex() {
		if (!semaphore) {
			semaphore = xSemaphoreCreateMutex();
		}
	}

	// init_mutex MUST be called before this
	MutexLock<T> get_lock() {
		return MutexLock<T>(mutex_wait, &t, semaphore);
	}
};

#endif /* INC_CPP_FREERTOS_HELPERS_HPP_ */
