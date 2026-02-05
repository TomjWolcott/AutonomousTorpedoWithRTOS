/*
 * control_loops.cpp
 *
 *  Created on: Jan 26, 2026
 *      Author: tomwolcott
 */

#include "control_loops.hpp"
#include <cmath>

#define M_TAU 6.283185307179586476925286766559

static float mod2(float a, float b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

float RollCL::update(float target, float measured) {
	Instant now = getInstant();

	const float err_left = mod2(target - measured, M_TAU);
	const float err_right = mod2(measured - target, M_TAU);

	const float error = (err_left < err_right) ? err_left : -err_right;
	const float dt = 1e-6 * (float)(elapsed_us2(last_time, now));

	float output = 0.0;

	// proportional
	output += p * error;

	// integral
	integral += error * dt;
	integral = std::min(i_max, std::max(i_min, integral));
	output += i * integral;

	// derivative
	output += d * (error - last_error) / dt;

	// everything else
	output = std::min(max, std::max(min, output));

	last_time = now;
	last_error = error;

	return output;
}


