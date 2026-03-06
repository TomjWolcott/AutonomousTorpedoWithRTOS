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
	output += params.p * error;

	// integral
	integral += error * dt;
	integral = std::min(params.i_max, std::max(params.i_min, integral));
	output += params.i * integral;

	// derivative
	output += params.d * (error - last_error) / dt;

	// everything else
	output = std::min(params.max, std::max(params.min, output));

	last_time = now;
	last_error = error;

	return output;
}

const float FIX_ROLL_UPPER = 0.32 * M_PI;
const float FIX_ROLL_LOWER = 0.16 * M_PI;

// Derivation of math in docs/orientationCL_math.jpg
RPYOutputs OrientationCL::update(quat<float> target, quat<float> measured) {
	const Instant now = getInstant();
	const float dt = 1e-6 * (float)(elapsed_us2(last_time, now));
	vec<float,3> output = vec<float,3>{ 0, 0, 0 };

	const quat<float> body_target = target * conjugate(measured);

	// Fix pitch + yaw first
	vec<float,3> x_err = body_target * vec<float,3>{1, 0, 0};
	vec<float,3> y_err = body_target * vec<float,3>{0, 1, 0};

	vec<float,3> axis_major = (Y(x_err) == 0.0 && Z(x_err) == 0.0)
		? vec<float,3>{0, 0, 1}
		: normalized(vec<float,3>{0, -Z(x_err), Y(x_err)});

	vec<float,3> axis_minor = cross(axis_major, x_err);

	float major_angle_error = acos(X(x_err));
	float phi_major = atan2(Z(axis_major), Y(axis_major));
	float phi_minor = atan2(dot(y_err, axis_minor), dot(y_err, axis_major));

	float py_angle = phi_minor - M_PI / 2.0;
	float roll_error = mod2(phi_major - phi_minor + M_PI, 2.0*M_PI) - M_PI;

	// Then fix roll once pitch + yaw is close enough
	const float fix_roll_t = 0.0;//fmax(0.0, fmin(1.0, (major_angle_error - FIX_ROLL_LOWER) / (FIX_ROLL_UPPER - FIX_ROLL_LOWER)));

	const vec<float,3> error = vec<float,3>{
		sin(py_angle) * major_angle_error,
		cos(py_angle) * major_angle_error,
		fix_roll_t * roll_error
	};

	output += params.p * error;

	integral += error * dt;
	X(integral) = fmin(params.i_max, fmax(params.i_min, X(integral)));
	Y(integral) = fmin(params.i_max, fmax(params.i_min, Y(integral)));
	Z(integral) = fmin(params.i_max, fmax(params.i_min, Z(integral)));
	output += params.i * integral;

	X(output) += params.d * X(error - last_error) / dt;
	Y(output) += params.d * Y(error - last_error) / dt;
	if (fix_roll_t >= 0.95) {
		Z(output) += params.d * Z(error - last_error) / dt;
	}

	last_time = now;
	last_error = error;

	return (RPYOutputs){ X(output), Y(output), Z(output) };
}

OriAndSpeedOutput update(
	float yaw_target, float yaw_measured,
	float speed_target, float speed_measured,
	float depth_target, float depth_measured
) {
	OriAndSpeedOutput output = (OriAndSpeedOutput){ (quat<float>){ 1, 0, 0, 0 }, 0 };

	// adjust quat to point at yaw

	return output;
}

