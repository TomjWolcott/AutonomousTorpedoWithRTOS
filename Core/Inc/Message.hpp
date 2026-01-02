/*
 * Message.hpp
 *
 *  Created on: Jan 1, 2026
 *      Author: tomwolcott
 */

#ifndef INC_MESSAGE_HPP_
#define INC_MESSAGE_HPP_

#if __cplusplus

#include <vector>
#include <span>
#include <optional>
#include <string>

#include "main.h"
#include "AdcData.hpp"
#include "ak09940a.hpp"
#include "ICM42688.hpp"

#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 1

#define LARGEST_MESSAGE_TYPE_ID 4

enum MessageType {
	MESSAGE_TYPE_PING = 0, // Device and controller
	MESSAGE_TYPE_ACTION = 1, // Controller
	MESSAGE_TYPE_SEND_DATA = 2, // Device
	MESSAGE_TYPE_PING_WITH_MS = 3, // Device
	MESSAGE_TYPE_SEND_CONFIG = 4, // Device and controller
	MESSAGE_TYPE_TEXT = 5,

	MESSAGE_TYPE_INCORRECT_FORMAT = 255
};

// ---------------------------------------------------- Action stuff ---------------------
enum ActionType {
    ACTION_TYPE_NOOP = 0,
	ACTION_TYPE_CALIBRATE = 1,
	ACTION_TYPE_SPIN_MOTOR = 2,
	ACTION_TYPE_SEND_CONFIG = 3,

	ACTION_TYPE_NOT_AN_ACTION = 255
};

enum CalibrationType {
	CALIBRATION_TYPE_ACC = 0,
	CALIBRATION_TYPE_MAG = 1,
	CALIBRATION_TYPE_GYR = 2,
};

class CalibrationActionMsg {
private:
    std::span<const uint8_t> data;
public:
    CalibrationActionMsg(std::span<const uint8_t> data) : data(data) {}

	CalibrationType type();
	uint16_t waitMsAfterUnplug();
	uint16_t dataCollectionRateHz();
	uint16_t dataCollectionTimeMs();
};

class ActionMsg {
private:
    std::span<const uint8_t> data;
public:
    ActionMsg(std::span<const uint8_t> data) : data(data) {}

	ActionType type();
	CalibrationActionMsg as_calibration();
};

// ---------------------------------------------------- SendData stuff ---------------------
#define SendDataAdcData 0x01
#define SendDataMag 0x02
#define SendDataAccGyro 0x04
#define SendDataDepthTemp 0x08
#define SendDataLocalizedData 0x10
#define SendDataAirPressure 0x20
#define SendDataOtherInfo 0x40

struct OtherData {
	uint32_t timestamp_us;
	uint16_t rate_hz;

	OtherData(uint32_t timestamp_us, uint16_t rate_hz) : timestamp_us(timestamp_us), rate_hz(rate_hz) {}

	void into_message(std::vector<uint8_t> &data);
};


// ---------------------------------------------------- Message Class ---------------------
class Message {
public:
	std::vector<uint8_t> data;

	Message(Message& message);
	Message(std::vector<uint8_t> data) : data(data) {}

	// Messages to be received
	static Message receiveWait();
	static Message fromData(uint8_t *from_data, uint32_t len);

	// Messages to be sent
	static Message ping();
	static Message pingWithMs();
	static Message sendData(AdcData *adcData, AK09940A_Output *ak09940a_output, ICM42688_Data *icm42688_data, OtherData *other_data);

	// Message operations
	bool isValid();
	MessageType type();
	void printDataToDisplay(uint8_t x, uint8_t y, uint8_t rows, uint8_t cols);
	std::string typeToString();
	void send();

	/// Extract Message types, assume Message::type is ALWAYS checked first
	ActionMsg as_action();
};

extern "C" {
#endif

int putchar_override(int c);
int _write_override(int fd, char *ptr, int len);

void messagesRxTxInit();

void msgInterruptRxHandler(uint32_t size);
void msgInterruptTxHandler();

#if __cplusplus
}
#endif

#endif /* INC_MESSAGE_HPP_ */
