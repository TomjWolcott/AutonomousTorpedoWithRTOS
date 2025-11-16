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

namespace SetupMode {
	using namespace SetupMode;

	void searchingForConnection(void *parameters) {
		for (;;) {
			uint8_t uart_tx_data[] = PING_MESSAGE;
			HAL_UART_Transmit_IT(&huart4, uart_tx_data, 6);
			osDelay(2000);
		}
	}

	void unconnectedBlinkBlonk(void *parameters) {
		for (;;) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			osDelay(500);
		}
	}

	void respondToInput(void *parameters) {
		xRxHandlerTask = xTaskGetCurrentTaskHandle();

		ssd1306_DrawCircle(40, 16, 10, White);
		ssd1306_DrawCircle(44, 16, 10, White);
		ssd1306_UpdateScreen();

		uint32_t uartDataLen = 0;
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, UART_RX_BUFFER, UART_RX_BUFFER_LEN);

		for (;;) {
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

			HAL_UARTEx_ReceiveToIdle_IT(&huart4, UART_RX_BUFFER, UART_RX_BUFFER_LEN);
		}
	}
}
