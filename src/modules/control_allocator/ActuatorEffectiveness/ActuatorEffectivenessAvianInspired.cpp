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

#include "ActuatorEffectivenessAvianInspired.hpp"
#include <ControlAllocation/ControlAllocation.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>


using namespace matrix;

ActuatorEffectivenessAvianInspired::ActuatorEffectivenessAvianInspired(ModuleParams *parent)
	: ModuleParams(parent), _rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedForward),
	  _control_surfaces(this)
{
}

bool
ActuatorEffectivenessAvianInspired::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	ModuleParams::updateParams();

	// int attitude_updated = _vehicle_attitude_sub.updated();

	// if (!attitude_updated) {
	// 	PX4_ERR("Failed to update to vehicle_attitude");
	// } else {
	// 	PX4_INFO("Successfully updated vehicle_attitude");
	// }

	if (_vehicle_attitude_sub.update(&vehicle_attitude)){
		matrix::Eulerf attitude = matrix::Quatf(vehicle_attitude.q);
		vel_body.pitch_angle = math::degrees(attitude(1)); // Convert pitch to degrees
		vel_body.pitch_angle_rad = attitude(1);            // Store pitch in radians

		PX4_INFO("Computed Pitch (degrees): %f", vel_body.pitch_angle);
	} else {
		PX4_WARN("Vehicle attitude not updated");
		vel_body.pitch_angle = vel_body.pitch_angle; // Keep at value
		vel_body.pitch_angle_rad = vel_body.pitch_angle_rad;
	}


	// PX4_INFO("Computed Pitch (degrees): %f, %f, %f, %f", static_cast<double>(vehicle_attitude.q[0]), static_cast<double>(vehicle_attitude.q[1]), static_cast<double>(vehicle_attitude.q[2]), static_cast<double>(vehicle_attitude.q[3]));


	// MavlinkStreamAttitudeQuaternion
	// vel_body.pitch_angle_rad = _att_quat.pitchl;

	// attitude_estimator_q_main
	// vel_body.pitch_angle_rad = estimator.get_pitch_avian();

	extractBodyFrameVelocities();

    	// PX4_INFO("PITCH BLACK IS : %f", vel_body.pitch_angle_rad);

	// computePitchAnge();
    	ServoControl serv_ctrl = getServoControlData();

	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	_rotors.enablePropellerTorque(false);
	const bool rotors_added_successfully = _rotors.addActuators(configuration);
	_forwards_motors_mask = _rotors.getForwardsMotors();

	// Control Surfaces
	_first_control_surface_idx = configuration.num_actuators_matrix[0];
	// const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration); // communication work without any call function
	// const bool surfaces_added_successfully = _control_surfaces.addActuatorsavian(configuration, serv_ctrl);
	const bool surfaces_added_successfully = _control_surfaces.addActuatorsavian(configuration, vel_body, serv_ctrl);

	return (rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessAvianInspired::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);
}

void ActuatorEffectivenessAvianInspired::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	// apply flaps
	normalized_unsigned_setpoint_s flaps_setpoint;

	if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
		_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
	}

	// apply spoilers
	normalized_unsigned_setpoint_s spoilers_setpoint;

	if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
		_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
	}
}


/*Helper Functions*/
void ActuatorEffectivenessAvianInspired::extractBodyFrameVelocities()
{
    AirspeedValidator air_speed;

    // Initialize result velocities and validity
    vel_body.valid = false;

    // Retrieve linear acceleration
    sensor_accel_s accel_data;
    if (!_sensor_accel_sub.update(&accel_data)) {
        PX4_ERR("Failed to update linear acceleration data");
    }

//     pitch_black = air_speed.get_pitch_filtered();

//     int did_it = _sensor_accel_sub.updated();
//     PX4_INFO("DID I SUBSCRIBE : %d", did_it);

    // Update velocities directly
    vel_body.vx += static_cast<double>(accel_data.x) * static_cast<double>(DT);
    vel_body.vy += static_cast<double>(accel_data.y) * static_cast<double>(DT);
    vel_body.vz += static_cast<double>(accel_data.z) * static_cast<double>(DT);

    // Apply wind adjustment using pitch
    double wind_adjustment = air_speed.get_aspd_wind_value();
    vel_body.pitch_angle_rad = math::radians(5);
    vel_body.vx += static_cast<double>(cos(vel_body.pitch_angle_rad)) * wind_adjustment;
    vel_body.vz += static_cast<double>(sin(vel_body.pitch_angle_rad)) * wind_adjustment;

    // Calculate AoA (only if forward velocity is positive)
    if (abs(vel_body.vx) > 0) {
        // vel_body.angle_of_attack = vel_body.pitch_angle_rad; // Convert to degrees
        vel_body.angle_of_attack = atan2(vel_body.vz, vel_body.vx) * (180.0 / M_PI); // Convert to degrees
    } else {
        vel_body.angle_of_attack = 0.9559; // Null AoA if forward velocity is zero
    }

    // Mark as valid
    vel_body.valid = true;
}


// Function to update the ServoControl struct with the latest data from the topic
ServoControl ActuatorEffectivenessAvianInspired::getServoControlData()
{
    // Initialize the uORB message structure for the actuator_servos topic
    actuator_servos_s actuator_servos_data;

    // Initialize a ServoControl struct to hold the data
    ServoControl serv_ctrl;

    // Check if there's new data available and copy it to actuator_servos_data
    if (_actuator_servos_sub.update(&actuator_servos_data)) {
        // If new data is available, fill the serv_ctrl struct
        serv_ctrl.timestamp = actuator_servos_data.timestamp;
        serv_ctrl.timestamp_sample = actuator_servos_data.timestamp_sample;

        // Fill the control array
        for (size_t i = 0; i < ServoControl::NUM_CONTROLS; ++i) {
            serv_ctrl.control[i] = actuator_servos_data.control[i];
        }

        serv_ctrl.valid = true; // Mark the struct as valid since we received data
    } else {
        PX4_WARN("No new servo data available");
        serv_ctrl.valid = false; // Mark the struct as invalid if no new data was available
    }

    return serv_ctrl;
}
