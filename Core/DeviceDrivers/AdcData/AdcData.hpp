/*
 * AdcData.hpp
 *
 *  Created on: Nov 17, 2025
 *      Author: tomwolcott
 */

#ifndef DEVICEDRIVERS_ADCDATA_ADCDATA_HPP_
#define DEVICEDRIVERS_ADCDATA_ADCDATA_HPP_

#include "main.h"

#if __cplusplus
#include "cpp_freertos_helpers.hpp"

void initADC();

#define ADC1_LEN 4
#define ADC2_LEN 4

struct AdcBufferCopy {
	uint16_t copy[ADC1_LEN + ADC2_LEN] = { 0, 0, 0, 0,   0, 0, 0, 0 };
	uint32_t data_taken_timestamp_us;

	AdcBufferCopy() {
		data_taken_timestamp_us = 0;
	}
};

extern MutexLazy<AdcBufferCopy> adcBufferCopy;

class AdcData {
public:
	uint32_t data_taken_timestamp_us;
    uint16_t vref_mv;         // (adc1)
    uint16_t self_temp_C;     // (adc1)
    uint16_t batt_mv;         // (adc1)
    uint16_t batt_temp_mv;    // (adc2)

    uint16_t ipropis_mv[4];   // (adc2, adc2, adc2, adc1)

	static AdcData from_buffer();
	void into_message(uint8_t *msg_data);
};

extern "C" {
#endif
void adcInterruptHandler(ADC_HandleTypeDef *handle);

#if __cplusplus
}
#endif


#endif /* DEVICEDRIVERS_ADCDATA_ADCDATA_HPP_ */
