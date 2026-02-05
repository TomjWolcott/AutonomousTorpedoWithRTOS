/*
 * control_loops.hpp
 *
 *  Created on: Jan 26, 2026
 *      Author: tomwolcott
 */

#ifndef DEVICEDRIVERS_CONTROL_LOOPS_CONTROL_LOOPS_HPP_
#define DEVICEDRIVERS_CONTROL_LOOPS_CONTROL_LOOPS_HPP_

#include "main.h"

#define BIG_NUMBER (1000000.0)

class RollCL {
private:
	float p, i, d;
	float min = -BIG_NUMBER;
	float max = BIG_NUMBER;

	float last_error = 0.0;
	Instant last_time = {0, 0};
	float integral = 0.0;
	float i_min = -BIG_NUMBER;
	float i_max = BIG_NUMBER;
public:
	RollCL() {
		RollCL(0, 0, 0);
	}
	RollCL(float p, float i, float d)
		: p(p), i(i), d(d) {
		last_time = getInstant();
	}
	RollCL(float p, float i, float d, float min, float max, float i_min, float i_max)
		: p(p), i(i), d(d), min(min), max(max), i_min(i_min), i_max(i_max) {
		last_time = getInstant();
	}

	float update(float target, float measured);
};


#endif /* DEVICEDRIVERS_CONTROL_LOOPS_CONTROL_LOOPS_HPP_ */
