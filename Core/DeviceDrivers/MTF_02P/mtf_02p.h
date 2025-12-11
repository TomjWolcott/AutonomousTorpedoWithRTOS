/*
 *
 * MTF-02P Optical Flow sensor and TOF distance ranger driver
 *
 * Author: Tom Wolcott
 * Created: 12 May 2025
 * Docs: https://micoair.com/optical_range_sensor_mtf-02p/
 *
 */

#ifndef MTF_02P_UART_DRIVER
#define MTF_02P_UART_DRIVER

#include "stdint.h"
#include <string.h> // For memcpy
#include "stm32g4xx_hal.h"

typedef struct __attribute__((packed)) {
	uint32_t time;
	uint32_t distance;
	uint8_t dist_strength;
	uint8_t dist_precision;
	uint8_t dist_status;
	uint8_t reserved1;
	int16_t flow_vel_x;
	int16_t flow_vel_y;
	uint8_t flow_quality;
	uint8_t flow_status;
	uint16_t reserved2;
} OpticalFlowRaw_t;

typedef struct {
	int valid;
	uint32_t time; //  ms
	float distance; //  m
	uint8_t dist_strength;
	uint8_t dist_precision;
	uint8_t dist_status;
	float flow_vel_x;
	float flow_vel_y;
	float vel_x; //  m/s
	float vel_y; //  m/s
	uint8_t flow_quality;
	uint8_t flow_status;
} OpticalFlow_t;

#define MTP_02P_UART_PORT huart1
#define OPTFLOW_RX_BUFF_LEN (int)26

extern UART_HandleTypeDef MTP_02P_UART_PORT;
extern OpticalFlow_t optflow_data;

#define MTF_02P_OK 0
#define MTF_02P_ERR 1

int get_optical_flow_data(int attempts);

#endif
