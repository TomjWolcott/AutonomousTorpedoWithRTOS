/*
 * MotorControl.hpp
 *
 *  Created on: Jan 15, 2026
 *      Author: tomwolcott
 */

#ifndef DEVICEDRIVERS_MOTORCONTROL_MOTORCONTROL_HPP_
#define DEVICEDRIVERS_MOTORCONTROL_MOTORCONTROL_HPP_

#include "main.h"
#include "AdcData.hpp"
#include <optional>

enum GainselType {
	GainselHighCurrent = 0, // 4A limit
	GainselMedCurrent = 1,   // 800mA limit
	GainselLowCurrent = 2   // 160mA limit
};

struct MotorInput {
    float drive_speed = 0.0; // within range [-1.0, 1.0]
    GainselType gainsel = GainselLowCurrent;

    MotorInput() {}

    MotorInput(float drive_speed, GainselType gainsel) : drive_speed(drive_speed), gainsel(gainsel) {}
};

struct MotorStats {
	float current; // (Amps)
	float voltage; // (Volts)
	float power; // (Watts)

	MotorStats(float current, float voltage, float power)
		: current(current), voltage(voltage), power(power) {}
};

struct AllMotorStats {
	std::array<MotorStats,4> stats;

	AllMotorStats(std::array<MotorStats,4> stats) : stats(stats) {}

	void into_message(std::vector<uint8_t> &data);
};

enum SetMotorInputStatus {
	SetMotorInputStatus_Success = 0,
	SetMotorInputStatus_BadInput = 1,
	SetMotorInputStatus_Stalled = 2,
	SetMotorInputStatus_HAL_ERROR = 3,

	SetMotorInputStatus_Unset = 99
};

struct SetMotorInputResult {
    float duty_cycle = 0.0;
    GainselType gainsel = GainselHighCurrent;
    SetMotorInputStatus status = SetMotorInputStatus_Unset;
    bool kickstarted = false;

    SetMotorInputResult() {}
};

struct StallInfo {
	Instant last_checked;
	float stall_input_voltage;
	float stall_current;

	StallInfo(Instant last_checked, float stall_input_voltage, float stall_current)
		: last_checked(last_checked), stall_input_voltage(stall_input_voltage), stall_current(stall_current) {}
};

// Indexes the motor controlers on the pcb from right to left looking head on at the pcb
enum MotorIndex {
	M0 = 0,
	M1 = 1,
	M2 = 2,
	M3 = 3
};

// Looking at the torpedo
enum MotorId {
	M_TL = 0,
	M_TR = 1,
	M_BL = 2,
	M_BR = 3
};

// Looking at the torpedo from the front with the config assumption of
//    CW | CCW
//  -----+-----
//   CCW | CW
// Where CW/CCW is the correct direction to spin for the that motor to push the torpedo forward
// TODO: make it possible to set this from the website / a part of the config
struct MotorConfig {
	// Top left
	MotorIndex tl_motor_id = M3;
	float tl_multiplier = 1.0;
	// Top right
	MotorIndex tr_motor_id = M0;
	float tr_multiplier = -1.0;
	// Bottom left
	MotorIndex bl_motor_id = M2;
	float bl_multiplier = -1.0;
	// Bottom right
	MotorIndex br_motor_id = M1;
	float br_multiplier = 1.0;

	MotorConfig() {}

	MotorIndex motorIdToIndex(MotorId id);
	float motorIdMultiplier(MotorId id);
};

/// Singleton class to handle motor control
class MotorControl {
private:
	// In MotorIndex order
	std::optional<StallInfo> stall_info_opts[4] = { std::nullopt, std::nullopt, std::nullopt, std::nullopt };
	MotorInput current_inputs[4] = { MotorInput(), MotorInput(), MotorInput(), MotorInput() };

	MotorConfig config = MotorConfig();
	float max_speed = 1.0;
	float global_speed_multiplier = 1.0;
	float max_motor_current = 2.0;

	SetMotorInputResult raw_set_motor_speed(float speed, GainselType gainsel, MotorIndex index);
public:
	AdcData data;
	MotorControl() {}

	void initialize_pwm();

	float to_raw_drive_speed(float drive_speed, MotorId id);

	/// In MotorId order
	std::array<SetMotorInputResult, 4> set_motor_inputs(MotorInput inputs[4]);
	std::array<SetMotorInputResult, 4> set_motor_speeds(std::array<float,4> speeds);

	SetMotorInputResult set_motor_input(MotorInput input, MotorId id);
	SetMotorInputResult set_motor_speed(float driveSpeed, MotorId id);
	SetMotorInputResult set_motor_gainsel(GainselType gainsel, MotorId id);

	float estimated_true_batt_v();
	MotorStats get_motor_stats(MotorId id);
	AllMotorStats get_all_motor_stats();
};

// OLD c header vvvvv | New cpp header ^^^^^^
//#include "main.h"

//typedef enum {
//	M0 = 0,
//	M1 = 1,
//	M2 = 2,
//	M3 = 3
//} MotorId;

//typedef enum {
//	GainselHighCurrent = 0, // 4A limit
//	GainselMedCurrent = 1,   // 800mA limit
//	GainselLowCurrent = 2   // 160mA limit
//} GainselType;

//typedef struct {
//    float drive_speed; // within range [-1.0, 1.0]
//    GainselType gainsel;
//} MotorInput;

//typedef struct {
//    double m[4]; // Amps
//} ControlTargetDirect;
//
//typedef struct {
//    double forward; // Amps (ish)
//    double roll;    // Amps (ish)
//    double pitch;   // Amps (ish)
//    double yaw;     // Amps (ish)
//} ControlTargetFRPY;

//typedef struct {
//	MotorIndex top_left_id;
//	double top_left_multiplier;
//	MotorIndex top_right_id;
//	double top_right_multiplier;
//	MotorIndex bottom_left_id;
//	double bottom_left_multiplier;
//	MotorIndex bottom_right_id;
//	double bottom_right_multiplier;
//} MotorConfig;

//static const double MAX_MOTOR_CURRENT = 3.0; // A
//static const double MAX_TOTAL_CURRENT = 7.0; // A
//
//// To be updated whenever motor wires are connected (from the POV of looking at the torpedo head on)
//static MotorConfig MOTOR_CONFIG = {
//	.top_left_id = M0,
//	.top_left_multiplier = 1.0,
//	.top_right_id = M1,
//	.top_right_multiplier = 1.0,
//	.bottom_left_id = M2,
//	.bottom_left_multiplier = 1.0,
//	.bottom_right_id = M3,
//	.bottom_right_multiplier = 1.0,
//};
//
//typedef struct {
//    float duty_cycle;
//    int status;
//} SetMotorSpeedOutput;
//
//typedef struct {
//    float duty_cycle[4];
//    int status;
//} SetMotorSpeedOutputs;

//extern MotorInput MOTOR_INPUTS[4];

///// Assumes that you are passing in the (len=4) array of motor inputs
//SetMotorSpeedOutputs set_motor_speeds(MotorInput inputs[4]);
//
///// Assumes that you are passing in a reference to one motor input
//SetMotorSpeedOutput set_motor_speed(MotorInput *input, MotorIndex id);

//int set_motor_pwm(float input, MotorIndex id);
//
//int set_motor_gainsel(GainselType input, MotorIndex id);
//
//float compute_motor_current(uint16_t v_ipropi, MotorId id);


#endif /* DEVICEDRIVERS_MOTORCONTROL_MOTORCONTROL_HPP_ */
