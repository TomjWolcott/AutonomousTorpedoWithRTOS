/*
 * cpp_main.hpp
 *
 *  Created on: Oct 29, 2025
 *      Author: tomwolcott
 */

#ifndef INC_CPP_MAIN_HPP_
#define INC_CPP_MAIN_HPP_

#include "cmsis_os.h"
#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

const osThreadAttr_t cppMainTask_attributes = {
	.name = "cppMainTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

#if __cplusplus

extern "C" {
#endif

void cppMainTask(void *argument);


#if __cplusplus
}
#endif

#endif /* INC_CPP_MAIN_HPP_ */
