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

	double mapRange(double value, double input_min, double input_max, double output_min, double output_max);

	SimpleArray<double, 3> getDirectionVector(double angle_of_attack, double twist_angle);

	SimpleArray<double, 3> flatPlateForce(const SimpleArray<double, 3>& direction, const SimpleArray<double, 3>& velocity,
									double surface_area, double alpha);

	double liftCoefficient(double alpha);

	double dragCoefficient(double alpha);

	double toRadians(double degrees);

	double norm(const SimpleArray<double, 3>& vec);

	double computePitchAnge();

	SimpleArray<double, 3> normalize(const SimpleArray<double, 3>& vec);

	SimpleArray<double, 3> crossProduct(const SimpleArray<double, 3>& vec1, const SimpleArray<double, 3>& vec2);

	SimpleArray<double, 3> add(const SimpleArray<double, 3>& vec1, const SimpleArray<double, 3>& vec2);

	SimpleArray<double, 3> subtract(const SimpleArray<double, 3>& vec1, const SimpleArray<double, 3>& vec2);

	SimpleArray<double, 3> multiply(const SimpleArray<double, 3>& vec, double scalar);

	ServoControl getServoControlData();

	BodyFrameVelocities extractBodyFrameVelocities(double pitch);

private:
	ActuatorEffectivenessRotors _rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;

	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};
	uORB::Subscription _spoilers_setpoint_sub{ORB_ID(spoilers_setpoint)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};


	// variables for speed computations
	double last_vel_x = 0;
	double last_vel_y = 0;
	double last_vel_z = 0;
	double pitch = 0;

	int _first_control_surface_idx{0}; ///< applies to matrix 1

	uint32_t _forwards_motors_mask{};
};
