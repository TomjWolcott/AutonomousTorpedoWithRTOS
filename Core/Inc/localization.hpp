/*
 * localization.hpp
 *
 *  Created on: Jan 11, 2026
 *      Author: tomwolcott
 */

#ifndef INC_LOCALIZATION_HPP_
#define INC_LOCALIZATION_HPP_

#include "qvm_lite.hpp"
#include "main.h"
#include <vector>

using namespace boost::qvm;

struct EulerAngles {
	float roll;
	float pitch;
	float yaw;
};

struct LocalizationOutput {
	quat<float> orientation = identity_quat<float>();
	vec<float, 3> position = {0, 0, 0};

	EulerAngles asEulerAngles();
	void into_message(std::vector<uint8_t> &data);
};

class Localization {
private:
public:
	Localization() {}
};

struct ILocalizationMethod {
	virtual ~ILocalizationMethod() = default;

	virtual void reset() = 0;
	virtual void update(vec<float,3> a, vec<float,3> m, vec<float,3> g) = 0;
	virtual LocalizationOutput output() = 0;
};

class ComplementaryFilter final : ILocalizationMethod {
private:
	quat<float> ori;
	Instant prev_update_instant;
public:
	float tuning_parameter;

	ComplementaryFilter() {
		ori = identity_quat<float>();
		prev_update_instant = { 0, 0 };
		tuning_parameter = 0.98;
	}

	void reset() override;
	void update(vec<float,3> acc, vec<float,3> mag, vec<float,3> gyr) override;
	LocalizationOutput output() override;
};

#endif /* INC_LOCALIZATION_HPP_ */
