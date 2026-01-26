/*
 * localization.cpp
 *
 *  Created on: Jan 11, 2026
 *      Author: tomwolcott
 */

#include "localization.hpp"
#include <math.h>
#include <bit>

void boost::throw_exception(std::exception const & e){
	printf("boost::qvm ERROR!! %s", e.what());
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
EulerAngles LocalizationOutput::asEulerAngles() {
	quat<float> &q = orientation;
    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (S(q) * X(q) + Y(q) * Z(q));
    float cosr_cosp = 1 - 2 * (X(q) * X(q) + Y(q) * Y(q));
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = sqrt(1 + 2 * (S(q) * Y(q) - X(q) * Z(q)));
    float cosp = sqrt(1 - 2 * (S(q) * Y(q) - X(q) * Z(q)));
    angles.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (S(q) * Z(q) + X(q) * Y(q));
    float cosy_cosp = 1 - 2 * (Y(q) * Y(q) + Z(q) * Z(q));
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

void LocalizationOutput::into_message(std::vector<uint8_t> &data) {
	float floats[7] = {
		// quat follows ordering of https://www.npmjs.com/package/quaternion
		S(orientation),
		X(orientation),
		Y(orientation),
		Z(orientation),
		X(position),
		Y(position),
		Z(position)
	};

	for (int i = 0; i < 7; i++) {
		uint32_t uint32_data = std::bit_cast<uint32_t>(floats[i]);

		data.push_back((uint8_t)(uint32_data >> 24));
		data.push_back((uint8_t)((uint32_data >> 16) & 0xFF));
		data.push_back((uint8_t)((uint32_data >> 8) & 0xFF));
		data.push_back((uint8_t)(uint32_data & 0xFF));
	}
}

static quat<float> quat_from_acc_mag(vec<float, 3> a, vec<float, 3> m) {
	normalize(a);
	normalize(m);
	quat<float> q1 = identity_quat<float>();
	quat<float> q2 = identity_quat<float>();

	float acc_to_z_angle = acosf(-Z(a));
	vec<float,3> z_to_acc_axis;

	if (abs(Z(a)) < 0.9999) {
		z_to_acc_axis = normalized(vec<float,3>{-Y(a), X(a), 0});
		rotate(q1, z_to_acc_axis, acc_to_z_angle);
		m = q1 * m;
	}

	if (abs(Z(m)) < 0.9999) {
		float mag_to_x_angle = atan2(Y(m), X(m));
		rotate_z(q2, -mag_to_x_angle);
	}

	return q2 * q1;
}

void ComplementaryFilter::reset() {
	ori = identity_quat<float>();
	prev_update_instant = { 0, 0 };
}

void ComplementaryFilter::update(vec<float,3> a, vec<float,3> m, vec<float,3> gyr) {
	quat<float> ori_from_acc_mag = quat_from_acc_mag(a, m);
	now = getInstant();

	if (prev_update_instant.tick_ms == 0 && prev_update_instant.tick_us == 0) {
		ori = ori_from_acc_mag;
	} else {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);

		float dt_s = 0.0020940;//static_cast<float>(elapsed_us(prev_update_instant, now)) / 1e6;

		float gyro_mag = mag(gyr);
		float gyro_angle_rad = dt_s * gyro_mag * M_PI / 180;
		vec<float,3> gyro_axis = gyr / gyro_mag;

		quat<float> ori_from_gyro = ori;
		rotate(ori_from_gyro, gyro_axis, -gyro_angle_rad);

		ori = slerp180(ori_from_acc_mag, ori_from_gyro, tuning_parameter);

//		vel += a * dt_s;
//		pos += vel * dt_s;
	}

	prev = prev_update_instant;
	prev_update_instant = now;
}

LocalizationOutput ComplementaryFilter::output() {
	return (LocalizationOutput){ ori, pos, vel };
}
