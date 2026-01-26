/*
 * Message.cpp
 *
 *  Created on: Jan 1, 2026
 *      Author: tomwolcott
 */

#include "Message.hpp"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include <cstdio>
#include <cstring>

#define MSG_RX_DATA_LEN 300
static uint8_t messageDataRx[MSG_RX_DATA_LEN];
static Message mostRecentMessageSent = Message(std::vector<uint8_t>());
EventGroupHandle_t xMsgEventGroup;
#define MSG_EVENT_GROUP_BIT (0x01)

#define MSG_TX_DATA_LEN 300
static uint8_t messageDataTx[MSG_TX_DATA_LEN];
SemaphoreHandle_t xMessageDataTxBusySemaphore;
static uint8_t MESSAGE_HEADER[4] = {0x11, 0x0F, 0xFF, 0x00};

// ----------------------------------- Messages via printf -----------------------------
#include "main.h"
#include <stdio.h>
#include "portmacro.h"

int putchar_override(int c) {
	_write_override(0, (char *)(&c), 1);

	return c;
}

int _write_override(int fd, char *ptr, int len) {
	if (!xPortIsInsideInterrupt()) {
		xSemaphoreTake(xMessageDataTxBusySemaphore, portMAX_DELAY);
	} else {
		BaseType_t _xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreTakeFromISR(xMessageDataTxBusySemaphore, &_xHigherPriorityTaskWoken);
	}
	messageDataTx[0] = MESSAGE_HEADER[0];
	messageDataTx[1] = MESSAGE_HEADER[1];
	messageDataTx[2] = MESSAGE_HEADER[2];
	messageDataTx[3] = MESSAGE_HEADER[3];
	messageDataTx[4] = 6 + len;
	messageDataTx[5] = MESSAGE_TYPE_TEXT;
	std::copy(ptr, ptr + len, messageDataTx + 6);
	HAL_UART_Transmit_IT(&huart4, messageDataTx, len + 6);

	return len;
}

// ----------------------------------- Normal Messages ---------------------------------

Message Message::receiveWait() {
	xEventGroupWaitBits(xMsgEventGroup, MSG_EVENT_GROUP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

	return Message(mostRecentMessageSent);
}

Message Message::receiveWait(std::function<bool(Message &)> isExpectedMessage) {
	Message msg = receiveWait();

	while (!isExpectedMessage(msg)) {
		msg = receiveWait();
	}

	return msg;
}

std::optional<Message> Message::receiveWait(std::function<bool(Message &)> isExpectedMessage, uint32_t timeout) {
	uint32_t start = HAL_GetTick();
	std::optional<Message> msg_opt = receiveWait(timeout);

	while (msg_opt.has_value() && !isExpectedMessage(msg_opt.value())) {
		uint32_t dt = HAL_GetTick() - start;
		msg_opt = receiveWait(timeout - dt);
	}

	return msg_opt;
}

std::optional<Message> Message::receiveWait(uint32_t timeout) {
	EventBits_t bits = xEventGroupWaitBits(xMsgEventGroup, MSG_EVENT_GROUP_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout));

	if ((bits & MSG_EVENT_GROUP_BIT) != 0) {
		return std::optional{mostRecentMessageSent};
	} else {
		return std::nullopt;
	}
}

Message Message::fromData(uint8_t *from_data, uint32_t len) {
	std::vector<uint8_t> data;
	uint8_t size = from_data[4];

	data.assign(from_data, from_data + (uint32_t)(size));

	return Message(data);
}

Message Message::ping() {
	std::vector<uint8_t> data = {MESSAGE_HEADER[0], MESSAGE_HEADER[1], MESSAGE_HEADER[2], MESSAGE_HEADER[3], 6, MESSAGE_TYPE_PING};

	return Message(data);
}

Message Message::pingWithMs() {
	uint32_t ms =  HAL_GetTick();

	std::vector<uint8_t> data = {
		MESSAGE_HEADER[0], MESSAGE_HEADER[1], MESSAGE_HEADER[2], MESSAGE_HEADER[3],
		10, MESSAGE_TYPE_PING_WITH_MS,
		(uint8_t)(ms >> 24),
		(uint8_t)((ms >> 16) & 0xFF),
		(uint8_t)((ms >> 8) & 0xFF),
		(uint8_t)(ms & 0xFF)
	};

	return Message(data);
}

void OtherData::into_message(std::vector<uint8_t> &data) {
	data.push_back(timestamp_us >> 24);
	data.push_back((timestamp_us >> 16) & 0xFF);
	data.push_back((timestamp_us >> 8) & 0xFF);
	data.push_back(timestamp_us & 0xFF);

	data.push_back(rate_hz >> 8);
	data.push_back(rate_hz & 0xFF);

	data.push_back((uint8_t)FIRMWARE_VERSION_MAJOR);
	data.push_back((uint8_t)FIRMWARE_VERSION_MINOR);
}

Message Message::sendData(
	std::optional<AdcData> adcData,
	std::optional<AK09940A_Output> ak09940a_output,
	std::optional<ICM42688_Data> icm42688_data,
	std::optional<OtherData> other_data,
	std::optional<LocalizationOutput> localization_output,
	std::optional<AllMotorStats> motor_stats
) {
	std::vector<uint8_t> data = {
		MESSAGE_HEADER[0],
		MESSAGE_HEADER[1],
		MESSAGE_HEADER[2],
		MESSAGE_HEADER[3],
		0, // Will be overwritten later
		MESSAGE_TYPE_SEND_DATA,
		0 // Will be overwritten later
	};

	if (adcData.has_value()) {
		adcData.value().into_message(data);
		data[6] |= SendDataAdcData;
	}

	if (ak09940a_output.has_value()) {
		ak09940a_output.value().into_message(data);
		data[6] |= SendDataMag;
	}

	if (icm42688_data.has_value()) {
		icm42688_data.value().into_message(data);
		data[6] |= SendDataAccGyro;
	}

	if (localization_output.has_value()) {
		localization_output.value().into_message(data);
		data[6] |= SendDataLocalizedData;
	}

	if (motor_stats.has_value()) {
		motor_stats.value().into_message(data);
		data[6] |= SendDataMotorData;
	}

	if (other_data.has_value()) {
		other_data.value().into_message(data);
		data[6] |= SendDataOtherInfo;
	}

	data[4] = data.size();
	
	return Message(data);
}

Message Message::sendCalibrationData(std::span<std::array<float, 3>> other_data, bool is_finished) {
	std::vector<uint8_t> data = {
		MESSAGE_HEADER[0],
		MESSAGE_HEADER[1],
		MESSAGE_HEADER[2],
		MESSAGE_HEADER[3],
		0, // Will be overwritten later
		MESSAGE_TYPE_CALIBRATION_DATA,
		(uint8_t)(is_finished ? 1 : 0)
	};

	for (std::array<float, 3> vector : other_data) {
		for (int i = 0; i < 3; i++) {
			uint32_t data_val = std::bit_cast<uint32_t>(vector[i]);
			data.push_back((uint8_t)(data_val >> 24));
			data.push_back((uint8_t)((data_val >> 16) & 0xFF));
			data.push_back((uint8_t)((data_val >> 8) & 0xFF));
			data.push_back((uint8_t)(data_val & 0xFF));
		}
	}

	data[4] = data.size();

	return Message(data);
}

Message Message::sendConfig(Config &config) {
	std::vector<uint8_t> data = {
		MESSAGE_HEADER[0],
		MESSAGE_HEADER[1],
		MESSAGE_HEADER[2],
		MESSAGE_HEADER[3],
		0, // Will be overwritten later
		MESSAGE_TYPE_SEND_CONFIG
	};

	config.into_msg_data(data);

	data[4] = data.size();

	return Message(data);
}

Message Message::echo() {
	return echo(std::vector<uint8_t>());
}

Message Message::echo(std::vector<uint8_t> v) {
	std::vector<uint8_t> data = {
		MESSAGE_HEADER[0],
		MESSAGE_HEADER[1],
		MESSAGE_HEADER[2],
		MESSAGE_HEADER[3],
		0, // Will be overwritten later
		MESSAGE_TYPE_ECHO,
		ECHO_ORIGIN_DEVICE
	};

	data.insert(data.end(), v.begin(), v.end());

	data[4] = data.size();

	return Message(data);
}

bool Message::isValid() {
	return (data.size() >= 6 &&
		data[0] == MESSAGE_HEADER[0] &&
		data[1] == MESSAGE_HEADER[1] &&
		data[2] == MESSAGE_HEADER[2] &&
		data[3] == MESSAGE_HEADER[3] &&
		data[4] == data.size() &&
		data[5] <= LARGEST_MESSAGE_TYPE_ID
	);
}

MessageType Message::type() {
	if (data.size() < 5) {
		return MESSAGE_TYPE_INCORRECT_FORMAT;
	} else {
		return (MessageType)(data[5]);
	}
}

void Message::printDataToDisplay(uint8_t x, uint8_t y, uint8_t rows, uint8_t cols) {
	ssd1306_FillRectangle(x, y, 3*6*cols + x, 8*rows + y, Black);

	int start = 0;
	char oled_s[100];

	if (isValid()) {
		sprintf(oled_s, " O:");
		start = 4;
	} else {
		sprintf(oled_s, " X:");
	}

	uint32_t index = start;

	ssd1306_SetCursor(x, y);
	for (int colIndex = 0; colIndex < cols-1 && index < data.size(); colIndex++) {
		if (index >= data.size()) break;
		sprintf(oled_s + strlen(oled_s), "%02X,", data[index]);
		index++;
	}

	for (int rowIndex = 1; rowIndex < rows && index < data.size(); rowIndex++) {
		ssd1306_WriteString(oled_s, Font_6x8, White);
		ssd1306_SetCursor(x, y + 8*rowIndex);
		oled_s[0] = 0;

		for (int colIndex = 0; colIndex < cols && index < data.size(); colIndex++) {
			sprintf(oled_s + strlen(oled_s), "%02X,", data[index]);
			index++;
		}
	}

	ssd1306_WriteString(oled_s, Font_6x8, White);

	ssd1306_UpdateScreen();
}

std::string Message::typeToString() {
	switch (type()) {
		case MESSAGE_TYPE_PING:
			return "PING";
		case MESSAGE_TYPE_ACTION:
			return "ACTION";
		case MESSAGE_TYPE_SEND_DATA:
			return "SEND_DATA";
		case MESSAGE_TYPE_PING_WITH_MS:
			return "PING_WITH_MS";
		case MESSAGE_TYPE_SEND_CONFIG:
			return "SEND_CONFIG";
		case MESSAGE_TYPE_TEXT:
			return "TEXT";
		case MESSAGE_TYPE_ECHO:
			return "ECHO";
		default:
			return "INCORRECT_FORMAT";
	}
}

std::string Message::dataToString() {
	std::string s = "[ ";
	for (uint8_t byte : data) {
		char byte_s[4];
		sprintf(byte_s, "%02X ", byte);
		s += byte_s;
	}

	return s + "]";
}

void Message::send() {
	xSemaphoreTake(xMessageDataTxBusySemaphore, portMAX_DELAY);
	copy(data.begin(), data.end(), messageDataTx);
	HAL_UART_Transmit_IT(&huart4, messageDataTx, messageDataTx[4]);
}

void messagesRxTxInit() {
	xMsgEventGroup = xEventGroupCreate();
	HAL_UARTEx_ReceiveToIdle_IT(&huart4, messageDataRx, MSG_RX_DATA_LEN);

	xMessageDataTxBusySemaphore =  xSemaphoreCreateBinary();
	xSemaphoreGive(xMessageDataTxBusySemaphore);
}

void msgInterruptRxHandler(uint32_t size) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	mostRecentMessageSent = Message::fromData(messageDataRx, MSG_RX_DATA_LEN);
	BaseType_t xResult = xEventGroupSetBitsFromISR(xMsgEventGroup, MSG_EVENT_GROUP_BIT, &xHigherPriorityTaskWoken);
	if( xResult != pdFAIL ) {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

	HAL_UARTEx_ReceiveToIdle_IT(&huart4, messageDataRx, MSG_RX_DATA_LEN);
}

void msgInterruptTxHandler() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xMessageDataTxBusySemaphore, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// Action
ActionMsg Message::asAction() {
	return ActionMsg(data);
}

ActionType ActionMsg::type() {
	return (ActionType)(data[6]);
}

CalibrationSettings ActionMsg::asCalibrationSettings() {
	CalibrationSettings settings = {};

	settings.type = (CalibrationType)(data[7]);
	settings.startSignal = (CalibrationStartSignal)(data[8]);
	settings.waitMsAfterUnplug = (uint16_t)(data[9] << 8) | (uint16_t)(data[10]);
	settings.dataCollectRateHz = (uint16_t)(data[11] << 8) | (uint16_t)(data[12]);
	settings.dataCollectTimeMs = (uint16_t)(data[13] << 8) | (uint16_t)(data[14]);

	return settings;
}

CalibrationMsgType ActionMsg::asCalibrationMsg() {
	return (CalibrationMsgType)(data[7]);
}

MotorSpeeds ActionMsg::asMotorSpeeds() {
	return MotorSpeeds(
		static_cast<float>(data[7]) / 127.0 - 1.0,
		static_cast<float>(data[8]) / 127.0 - 1.0,
		static_cast<float>(data[9]) / 127.0 - 1.0,
		static_cast<float>(data[10]) / 127.0 - 1.0
	);
}

// Echo
EchoOrigin Message::asEchoOrigin() {
	return (EchoOrigin)(data[6]);
}

const uint8_t DEVICE_CONNECTED_CHECK_DATA[] = {0x12, 0x34, 0x56};

bool isDeviceConnected(uint32_t echo_timeout) {
	std::vector<uint8_t> data = {0x12, 0x34, 0x56};
	Message echo_out = Message::echo(data);

	echo_out.send();

	return Message::receiveWait([&echo_out](Message &msg) {
		return msg.data == echo_out.data;
	}, echo_timeout).has_value();
}

// Config
Config Message::asConfig() {
	return Config::from_msg_data(data);
}
