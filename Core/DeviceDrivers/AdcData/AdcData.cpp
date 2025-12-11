/*
 * AdcData.cpp
 *
 *  Created on: Nov 17, 2025
 *      Author: tomwolcott
 */

#include "AdcData.hpp"
#include <cstring>

static uint16_t adcBuffer1[ADC1_LEN] = {};
static uint16_t adcBuffer2[ADC2_LEN] = {};
MutexLazy<AdcBufferCopy> adcBufferCopy = MutexLazy<AdcBufferCopy>(AdcBufferCopy());

void initADC() {
	adcBufferCopy.ensureInitialized();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer1, ADC1_LEN);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcBuffer2, ADC2_LEN);
}

extern "C" void adcInterruptHandler(ADC_HandleTypeDef *handle) {
	if (handle->Instance == ADC1) {
		auto lock = adcBufferCopy.get_lock();
		std::memcpy(lock->copy, adcBuffer1, ADC1_LEN * sizeof(uint16_t));
		lock->data_taken_timestamp_us = 1000 * HAL_GetTick();
		lock.unlock();
	} else if (handle->Instance == ADC2) {
		auto lock = adcBufferCopy.get_lock();
		std::memcpy(lock->copy+ADC1_LEN, adcBuffer2, ADC2_LEN * sizeof(uint16_t));
		lock->data_taken_timestamp_us = 1000 * HAL_GetTick();
		lock.unlock();
	}
}

#define ADC_RES ADC_RESOLUTION_12B

AdcData AdcData::from_buffer() {
	AdcData adc_data = AdcData();

	auto lock = adcBufferCopy.get_lock();
	adc_data.data_taken_timestamp_us = lock->data_taken_timestamp_us;
	uint16_t *copy_data = lock->copy;

//	uint16_t VREFINT_CAL = *((uint16_t*)VREFINT_CAL_ADDR);

	adc_data.vref_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(copy_data[0], ADC_RES);

	adc_data.self_temp_C = __HAL_ADC_CALC_TEMPERATURE(adc_data.vref_mv, copy_data[1], ADC_RES);
	adc_data.batt_mv = 3 * __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_data.vref_mv, copy_data[3], ADC_RES);
	adc_data.batt_temp_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_data.vref_mv, copy_data[6], ADC_RES);

	adc_data.ipropis_mv[0] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_data.vref_mv, copy_data[4], ADC_RES);
	adc_data.ipropis_mv[1] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_data.vref_mv, copy_data[7], ADC_RES);
	adc_data.ipropis_mv[2] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_data.vref_mv, copy_data[5], ADC_RES);
	adc_data.ipropis_mv[3] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(adc_data.vref_mv, copy_data[2], ADC_RES);

	lock.unlock();

	return adc_data;
}

/// Requires 18 bytes starting at msg_data
void AdcData::into_message(uint8_t *msg_data) {
	msg_data[0] = (uint8_t)(data_taken_timestamp_us >> 24);
	msg_data[1] = (uint8_t)((data_taken_timestamp_us >> 16) & 0xFF);
	msg_data[2] = (uint8_t)((data_taken_timestamp_us >> 8) & 0xFF);
	msg_data[3] = (uint8_t)(data_taken_timestamp_us & 0xFF);

	msg_data[4] = (uint8_t)(vref_mv >> 8);
	msg_data[5] = (uint8_t)(vref_mv & 0xFF);

	msg_data[6] = (uint8_t)(self_temp_C >> 8);
	msg_data[7] = (uint8_t)(self_temp_C & 0xFF);

	msg_data[8] = (uint8_t)(batt_mv >> 8);
	msg_data[9] = (uint8_t)(batt_mv & 0xFF);

	for (int i = 0; i < 4; i++) {
		msg_data[10 + 2*i] = (uint8_t)(ipropis_mv[i] >> 8);
		msg_data[11 + 2*i] = (uint8_t)(ipropis_mv[i] & 0xFF);
	}
}
