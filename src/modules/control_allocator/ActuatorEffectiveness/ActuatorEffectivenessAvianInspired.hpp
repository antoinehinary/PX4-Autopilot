/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"
// #include "/home/antoine/PX4-Autopilot/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp"

#include <uORB/topics/normalized_unsigned_setpoint.h>

class ActuatorEffectivenessAvianInspired : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessAvianInspired(ModuleParams *parent);
	virtual ~ActuatorEffectivenessAvianInspired() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	const char *name() const override { return "Fixed Wing"; }

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	ServoControl getServoControlData();

	void extractBodyFrameVelocities();


private:
	ActuatorEffectivenessRotors _rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;
	BodyFrameVelocities vel_body;

	vehicle_attitude_s vehicle_attitude{};

	// AttitudeEstimatorQ estimator;

	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};
	uORB::Subscription _spoilers_setpoint_sub{ORB_ID(spoilers_setpoint)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// MavlinkStreamAttitudeQuaternion _att_quat;

	// variables for speed computations
	double last_vel_x = 0;
	double last_vel_y = 0;
	double last_vel_z = 0;

	float pitch_black = 0;

	int _first_control_surface_idx{0}; ///< applies to matrix 1

	uint32_t _forwards_motors_mask{};
};
