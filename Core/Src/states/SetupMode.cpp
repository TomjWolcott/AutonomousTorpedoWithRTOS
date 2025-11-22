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

static uint32_t stack_expense[3] = {0, 0, 0};

namespace SetupMode {
	using namespace SetupMode;

	void __NO_RETURN searchingForConnection(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			stack_expense[0] = 4 * uxTaskGetStackHighWaterMark(NULL);
			uint32_t ms = HAL_GetTick();
			uint8_t uart_tx_data[] = PING_WITH_MS(ms);

			xSemaphoreTake(xUart4TxBusySemaphore, portMAX_DELAY);
			HAL_UART_Transmit_IT(&huart4, uart_tx_data, uart_tx_data[4]);
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

		xRxHandlerTask = xTaskGetCurrentTaskHandle();

//		ssd1306_DrawCircle(40, 16, 10, White);
//		ssd1306_DrawCircle(44, 16, 10, White);
//		ssd1306_UpdateScreen();

		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("2025 Fall", Font_6x8, White);
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString("Design Review", Font_6x8, White);
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("AutonoTorpedo V1.3", Font_6x8, White);
		ssd1306_UpdateScreen();

		uint32_t uartDataLen = 0;
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, UART_RX_BUFFER, UART_RX_BUFFER_LEN);

		while (!this_task->is_task_dead) {
			BaseType_t xResult = xTaskNotifyWait( pdFALSE, UINT32_MAX, &uartDataLen, portMAX_DELAY);

			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			int start = 0;
			char oled_s[100];
			if (IS_VALID_MESSAGE(UART_RX_BUFFER, uartDataLen)) {
				sprintf(oled_s, "O[");
				start = 4;
			} else {
				sprintf(oled_s, "X[");
			}

			for (uint32_t i = start; i < uartDataLen && strlen(oled_s) < 27; i++) {
				sprintf(oled_s + strlen(oled_s), "%02X,", UART_RX_BUFFER[i]);
			}

			sprintf(oled_s + strlen(oled_s), "]");
			ssd1306_WriteString(oled_s, Font_6x8, White);
			ssd1306_UpdateScreen();
			uartDataLen = 0;

			ssd1306_SetCursor(0, 10);
			sprintf(oled_s, "");
			for (uint32_t i = 0; i < 3; i++) {
				sprintf(oled_s + strlen(oled_s), "%d, ", stack_expense[i]);
			}
			ssd1306_WriteString(oled_s, Font_6x8, White);
			ssd1306_UpdateScreen();

			HAL_UARTEx_ReceiveToIdle_IT(&huart4, UART_RX_BUFFER, UART_RX_BUFFER_LEN);

			auto lock = systemModesSM.get_lock();

			if (lock->is<decltype(sml::state<SM>)>(sml::state<Unconnected>)) {
				lock->process_event(EnterConnected {});
			}

			lock.unlock();
			stack_expense[2] = 4*uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	void __NO_RETURN collectData(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			auto lock = dataMutex.get_lock();
			lock->adcData = AdcData::from_buffer();
			lock->ak09940a_dev.get_avg_measurement(5, &(lock->ak09940a_output));
			lock->icm42688_output = lock->icm42688_dev.get_avg_data(5);
			lock.unlock();
			osDelay(100);
			stack_expense[0] = 4 * uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}

	void __NO_RETURN sendData(void *parameters) {
		Task *this_task = (Task *)parameters;

		while (!this_task->is_task_dead) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			auto lock = dataMutex.get_lock();
			xSemaphoreTake(xUart4TxBusySemaphore, portMAX_DELAY);

			#define DATA_LEN 49
			uint8_t data[DATA_LEN] = SEND_DATA_MESSAGE(DATA_LEN);
			data[6] = 0x01 | 0x02 | 0x04; // TODO: Convert into define flags
			lock->adcData.into_message(data+7);
			lock->ak09940a_output.into_message(data+25);
			lock->icm42688_output.into_message(data+37);
			HAL_UART_Transmit_IT(&huart4, data, DATA_LEN);

			lock.unlock();
			osDelay(100);
			stack_expense[1] = 4 * uxTaskGetStackHighWaterMark(NULL);
		}

		osThreadExit();
	}
}
