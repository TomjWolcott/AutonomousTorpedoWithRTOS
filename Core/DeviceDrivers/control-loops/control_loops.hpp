/*
 * control_loops.hpp
 *
 *  Created on: Jan 26, 2026
 *      Author: tomwolcott
 */

#ifndef DEVICEDRIVERS_CONTROL_LOOPS_CONTROL_LOOPS_HPP_
#define DEVICEDRIVERS_CONTROL_LOOPS_CONTROL_LOOPS_HPP_

#include "main.h"
#include "qvm_lite.hpp"

using namespace boost::qvm;

#define BIG_NUMBER (1000000.0)

struct PidParams {
	float p, i, d;
	float min = -BIG_NUMBER;
	float max = BIG_NUMBER;
	float i_min = -BIG_NUMBER;
	float i_max = BIG_NUMBER;

	PidParams() {
		p = 0;
		i = 0;
		d = 0;
	}

	PidParams(float p, float i, float d)
		: p(p), i(i), d(d) {}

	PidParams(float p, float i, float d, float min, float max, float i_min, float i_max)
		: p(p), i(i), d(d), min(min), max(max), i_min(i_min), i_max(i_max) {}
};

class RollCL {
private:
	PidParams params;

	float last_error = 0.0;
	Instant last_time = {0, 0};
	float integral = 0.0;
public:
	RollCL() {

	}
	RollCL(PidParams params) : params(params) {
		last_time = getInstant();
	}

	float update(float target, float measured);
};

struct RPYOutputs {
	float roll;
	float pitch;
	float yaw;
};

class OrientationCL {
private:
	PidParams params;

	vec<float,3> last_error = {0, 0, 0};
	Instant last_time = {0, 0};
	vec<float,3> integral = {0, 0, 0};
public:
	OrientationCL() {}
	OrientationCL(PidParams params) : params(params) {
		last_time = getInstant();
	}

	RPYOutputs update(quat<float> target, quat<float> measured);
};

struct OriAndSpeedOutput {
	quat<float> ori;
	float forward;
};

class DepthAndVelCL {
private:
	PidParams vel_params;
	PidParams depth_params;

	float last_error = 0.0;
	Instant last_time = {0, 0};
	vec<float,2> vel_integral = {0.0, 0.0};
	float depth_integral = 0.0;
public:
	DepthAndVelCL() {}
	DepthAndVelCL(PidParams vel_params, PidParams depth_params) : vel_params(vel_params), depth_params(depth_params) {
		last_time = getInstant();
	}

	OriAndSpeedOutput update(
		float yaw_target, float yaw_measured,
		float speed_target, float speed_measured,
		float depth_target, float depth_measured
	);
};

#endif /* DEVICEDRIVERS_CONTROL_LOOPS_CONTROL_LOOPS_HPP_ */
