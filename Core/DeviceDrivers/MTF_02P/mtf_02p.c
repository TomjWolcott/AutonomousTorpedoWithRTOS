#include "mtf_02p.h"

uint8_t optflow_rx_buff[OPTFLOW_RX_BUFF_LEN];
OpticalFlowRaw_t optflow_data_raw;
OpticalFlow_t optflow_data;

int get_optical_flow_data(int attempts) {
	while (attempts--) {
		int uart_recieve_code = HAL_UART_Receive(&MTP_02P_UART_PORT, optflow_rx_buff, OPTFLOW_RX_BUFF_LEN, 1000);

		if (optflow_rx_buff[0] != 0xef || uart_recieve_code != HAL_OK) {
			continue;
		}

		memcpy(&optflow_data_raw, optflow_rx_buff + 6, sizeof(optflow_data_raw));
		break;
	}

	if (attempts == 0) {
		return MTF_02P_ERR;
	}

	optflow_data.time = optflow_data_raw.time;
	optflow_data.distance = (float)optflow_data_raw.distance / 1000.0;
	optflow_data.dist_strength = optflow_data_raw.dist_strength;
	optflow_data.dist_precision = optflow_data_raw.dist_precision;
	optflow_data.dist_status = optflow_data_raw.dist_status;
	optflow_data.flow_vel_x = (float)optflow_data_raw.flow_vel_x;
	optflow_data.flow_vel_y = (float)optflow_data_raw.flow_vel_y;
	optflow_data.vel_x = (optflow_data.flow_vel_x * optflow_data.distance) / 100.0;
	optflow_data.vel_y = (optflow_data.flow_vel_y * optflow_data.distance) / 100.0;
	optflow_data.flow_quality = optflow_data_raw.flow_quality;
	optflow_data.flow_status = optflow_data_raw.flow_status;

	return MTF_02P_OK;
}
