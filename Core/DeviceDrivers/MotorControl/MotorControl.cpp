/*
 * MotorControl.cpp
 *
 *  Created on: Jan 15, 2026
 *      Author: tomwolcott
 */

#include "MotorControl.hpp"
#include <cmath>
#include <cstdio>

void AllMotorStats::into_message(std::vector<uint8_t> &data) {
	uint8_t ordered_data[16] = { 0 };

	for (int i = 0; i < 4; i++) {
		uint32_t current_data = static_cast<uint32_t>(65536.0 * stats[i].current / 8.0);
		uint32_t voltage_data = static_cast<uint32_t>(65536.0 * stats[i].voltage / 8.0);

		ordered_data[2*i + 0] = ((current_data >> 8) & 0xFF);
		ordered_data[2*i + 1] = (current_data & 0xFF);

		ordered_data[2*i + 8] = ((voltage_data >> 8) & 0xFF);
		ordered_data[2*i + 9] = (voltage_data & 0xFF);

	}

	data.insert(data.end(), ordered_data, ordered_data + 16);
}

MotorIndex MotorConfig::motorIdToIndex(MotorId id) {
	switch (id) {
	case (M_TL):
		return tl_motor_id;
    case (M_TR):
    	return tr_motor_id;
    case (M_BL):
    	return bl_motor_id;
    case (M_BR):
    	return br_motor_id;
    default:
        return (MotorIndex)id; // Fallback to direct mapping
	}
}

float MotorConfig::motorIdMultiplier(MotorId id) {
    switch (id) {
    case (M_TL):
        return tl_multiplier;
    case (M_TR):
    	return tr_multiplier;
    case (M_BL):
    	return bl_multiplier;
    case (M_BR):
    	return br_multiplier;
    default:
        return 1.0; // Fallback to neutral multiplier
    }
}

void MotorControl::initialize_pwm() {
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	// M3 uses CH2N and CH3N which require different PWM start functions
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	MotorInput motor_inputs[4] = {
		MotorInput(0.0, GainselHighCurrent),
		MotorInput(0.0, GainselHighCurrent),
		MotorInput(0.0, GainselHighCurrent),
		MotorInput(0.0, GainselHighCurrent)
	};

    set_motor_inputs(motor_inputs);
}

SetMotorInputResult MotorControl::raw_set_motor_speed(float speed, GainselType gainsel, MotorIndex index) {
	SetMotorInputResult result;

	current_inputs[index] = MotorInput(speed, gainsel);

	if (stall_info_opts[index].has_value()) {
		result.status = SetMotorInputStatus_Stalled;

		return result;
	}

    TIM_HandleTypeDef *timer_f;
    TIM_HandleTypeDef *timer_r;
    uint32_t timer_channel_f;
    uint32_t timer_channel_r;

    GPIO_TypeDef *gainsel_port;
    uint16_t gainsel_pin;

    switch (index) {
        case M0:
            timer_f = &htim15;
            timer_channel_f = TIM_CHANNEL_1;
            timer_r = &htim15;
            timer_channel_r = TIM_CHANNEL_2;

            gainsel_port = m0_gainsel_GPIO_Port;
            gainsel_pin = m0_gainsel_Pin;
            break;
        case M1:
            timer_f = &htim3;
            timer_channel_f = TIM_CHANNEL_1;
            timer_r = &htim3;
            timer_channel_r = TIM_CHANNEL_2;

            gainsel_port = m1_gainsel_GPIO_Port;
            gainsel_pin = m1_gainsel_Pin;
            break;
        case M2:
            timer_f = &htim3;
            timer_channel_f = TIM_CHANNEL_3;
            timer_r = &htim2;
            timer_channel_r = TIM_CHANNEL_4;

            gainsel_port = m2_gainsel_GPIO_Port;
            gainsel_pin = m2_gainsel_Pin;
            break;
        case M3:
            timer_f = &htim1;
            timer_channel_f = TIM_CHANNEL_2;
            timer_r = &htim1;
            timer_channel_r = TIM_CHANNEL_3;

            gainsel_port = m3_gainsel_GPIO_Port;
            gainsel_pin = m3_gainsel_Pin;
            break;
        default:
        	result.status = SetMotorInputStatus_BadInput;
            return result;
    }

    int compare_value = (int)(1000.0 * (1.0 - std::abs(speed)));
    result.duty_cycle = (float)(1000 - compare_value) / 1000.0;

    int duty_active = compare_value;
    int duty_reference = 1000;

    if (speed > 0.0) {
        __HAL_TIM_SET_COMPARE(timer_f, timer_channel_f, duty_reference);
        __HAL_TIM_SET_COMPARE(timer_r, timer_channel_r, duty_active);
    } else {
        __HAL_TIM_SET_COMPARE(timer_f, timer_channel_f, duty_active);
        __HAL_TIM_SET_COMPARE(timer_r, timer_channel_r, duty_reference);
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gainsel_pin;

    // TODO: VERIFY THAT THIS ACTUALLY WORKS
    switch (gainsel) {
        case GainselHighCurrent:
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            break;
        case GainselMedCurrent:
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
        case GainselLowCurrent:
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            break;
        default:
        	result.status = SetMotorInputStatus_BadInput;
            return result;
    }

    result.gainsel = gainsel;

    HAL_GPIO_Init(gainsel_port, &GPIO_InitStruct);
    result.status = SetMotorInputStatus_Success;

    return result;
}

std::array<SetMotorInputResult, 4> MotorControl::set_motor_inputs(MotorInput inputs[4]) {
	std::array<SetMotorInputResult, 4> results;

	for (int i = 0; i < 4; i++)
		set_motor_input(inputs[i], (MotorId)i);

	return results;
}

std::array<SetMotorInputResult, 4> MotorControl::set_motor_speeds(std::array<float,4> speeds) {
	std::array<SetMotorInputResult, 4> results;

	for (int i = 0; i < 4; i++)
		set_motor_speed(speeds[i], (MotorId)i);

	return results;
}

static float fsign(float x) {
    return (x > 0.0) - (x < 0.0);
}

float MotorControl::to_raw_drive_speed(float drive_speed, MotorId id) {
	drive_speed *= global_speed_multiplier * config.motorIdMultiplier(id);

	if (std::abs(drive_speed) > max_speed) {
		drive_speed = fsign(drive_speed) * max_speed;
	}

	return drive_speed;
}

const float KICKSTART_SPEED_CUTOFF = 0.7;

SetMotorInputResult MotorControl::set_motor_input(MotorInput input, MotorId id) {
	SetMotorInputResult result;

    MotorIndex index = config.motorIdToIndex(id);
	input.drive_speed = to_raw_drive_speed(input.drive_speed, id);

//    if (std::abs(input.drive_speed) < KICKSTART_SPEED_CUTOFF && current_inputs[index].drive_speed * input.drive_speed <= 0.0) {
//    	result = raw_set_motor_speed(fsign(input.drive_speed), GainselHighCurrent, index);
//    	result.kickstarted = true;
//
//    	if (result.status != SetMotorInputStatus_Success) {
//    		return result;
//    	}
//
//    	osDelay(1);
//    }

    SetMotorInputResult other_result = raw_set_motor_speed(input.drive_speed, input.gainsel, index);

    result.duty_cycle = other_result.duty_cycle;
    result.gainsel = other_result.gainsel;
    result.status = other_result.status;

	return result;
}

SetMotorInputResult MotorControl::set_motor_speed(float drive_speed, MotorId id) {
	SetMotorInputResult result;

	// TODO: add additional logic here to switch gainsel if the speed requires a different gainsel
    MotorIndex index = config.motorIdToIndex(id);
	drive_speed = to_raw_drive_speed(drive_speed, id);

//	return raw_set_motor_speed(drive_speed, current_inputs[index].gainsel, index);
	return raw_set_motor_speed(drive_speed, GainselHighCurrent, index);
}

SetMotorInputResult MotorControl::set_motor_gainsel(GainselType gainsel, MotorId id) {
	SetMotorInputResult result;

    MotorIndex index = config.motorIdToIndex(id);

	return raw_set_motor_speed(current_inputs[index].drive_speed, gainsel, index);
}

float MotorControl::estimated_true_batt_v() {
	data = AdcData::from_buffer();

	return data.batt_v();
}

const float DRV8213_R_IPROPI = 677.0; // ohms
const float DRV8213_V_REF = 0.501; // volts
const float DRV8213_GAINS[3] = { 205.0, 1050.0, 4900.0 };

MotorStats MotorControl::get_motor_stats(MotorId id) {
	data = AdcData::from_buffer();
    MotorIndex index = config.motorIdToIndex(id);

    float motor_voltage = data.batt_v() * std::abs(current_inputs[index].drive_speed);
    std::array<float, 4> v_ipropi = data.ipropis_v();
//    printf("v_ipropi[index] = %.5f\n", v_ipropi[index]);
    float motor_current = 1000000.0 * v_ipropi[index] / (DRV8213_R_IPROPI * DRV8213_GAINS[current_inputs[index].gainsel]);
    float motor_power = motor_voltage * motor_current;

    return MotorStats(motor_current, motor_voltage, motor_power);
}

AllMotorStats MotorControl::get_all_motor_stats() {
	return AllMotorStats({
		get_motor_stats(M_TL),
		get_motor_stats(M_TR),
		get_motor_stats(M_BL),
		get_motor_stats(M_BR),
	});
}
