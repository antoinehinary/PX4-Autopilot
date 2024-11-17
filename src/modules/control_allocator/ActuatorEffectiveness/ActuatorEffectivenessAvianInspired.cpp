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
	double pitch = computePitchAnge();
    	ServoControl serv_ctrl = getServoControlData();
	BodyFrameVelocities vel_body = extractBodyFrameVelocities(pitch);
	// PX4_INFO("pitch computed %f", pitch);

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


/*Helper functions*/
double ActuatorEffectivenessAvianInspired::mapRange(double value, double input_min, double input_max,
		double output_min, double output_max)
{
	// Define a small epsilon value for floating-point comparison
	constexpr double epsilon = 1e-6;

	// Check if the input range is effectively zero
	if (fabs(input_max - input_min) < epsilon) {
		PX4_ERR("Invalid input range");
		return output_min; // Return output_min as a fallback
	}

	// Linearly map the value from the input range to the output range
	double scaled_value = (value - input_min) / (input_max - input_min);
	return output_min + scaled_value * (output_max - output_min);
}


SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::flatPlateForce(const SimpleArray<double, 3> &lift_dir,
		const SimpleArray<double, 3> &velocity,
		double surface_area, double alpha)
{
	// Define a small epsilon value for floating-point comparison
	constexpr double epsilon = 1e-6;

	if (fabs(surface_area) < epsilon || norm(velocity) < epsilon) {
		return {0, 0, 0};
	}

	// Normalize the velocity and compute lift and drag directions
	auto vel_norm = normalize(velocity);

	// Compute dynamic pressure
	double dynamic_pressure = 0.5 * AIR_DENSITY * std::pow(norm(velocity), 2);
	double cl = liftCoefficient(toRadians(alpha));
	double cd = dragCoefficient(toRadians(alpha));

	// Compute lift and drag forces
	auto lift_force = multiply(lift_dir, dynamic_pressure * surface_area * cl);
	auto drag_force = multiply(vel_norm, dynamic_pressure * surface_area * cd);

	// Return the total force (lift + drag)
	return add(lift_force, drag_force);
}

double ActuatorEffectivenessAvianInspired::liftCoefficient(double alpha)
{
	double cl_baseline = 0.1;
	return cl_baseline + 2 * std::sin(alpha) * std::cos(alpha);
}

double ActuatorEffectivenessAvianInspired::dragCoefficient(double alpha)
{
	double cd_baseline = 0.02;
	return cd_baseline + 2 * std::pow(std::sin(alpha), 2);
}

double ActuatorEffectivenessAvianInspired::toRadians(double degrees)
{
	return degrees * M_PI / 180.0;
}

double ActuatorEffectivenessAvianInspired::norm(const SimpleArray<double, 3> &vec)
{
	return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::crossProduct(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	return {
		vec1[1] *vec2[2] - vec1[2] *vec2[1],
		vec1[2] *vec2[0] - vec1[0] *vec2[2],
		vec1[0] *vec2[1] - vec1[1] *vec2[0]
	};
}

SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::add(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	return {vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2]};
}

SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::subtract(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	return {vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2]};
}

SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::multiply(const SimpleArray<double, 3> &vec, double scalar)
{
	return {vec[0] *scalar, vec[1] *scalar, vec[2] *scalar};
}

SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::normalize(const SimpleArray<double, 3> &vec)
{
	double vec_norm = norm(vec);
	constexpr double epsilon = 1e-9; // Small threshold value to avoid floating-point comparison

	if (std::fabs(vec_norm) < epsilon) {
		return {0.0, 0.0, 0.0};
	}

	return {vec[0] / vec_norm, vec[1] / vec_norm, vec[2] / vec_norm};
}


SimpleArray<double, 3> ActuatorEffectivenessAvianInspired::getDirectionVector(double angle_of_attack,
		double twist_angle)
{
	// Convert angles to radians
	double aoa_rad = toRadians(angle_of_attack);
	double twist_rad = toRadians(twist_angle);

	// Initial direction vector based on the angle of attack
	SimpleArray<double, 3> direction_vector = {std::sin(aoa_rad), 0, -std::cos(aoa_rad)};

	// Apply rotation for the twist angle (rotation about the x-axis)
	SimpleArray<SimpleArray<double, 3>, 3> twist_rotation = {{
			{1, 0, 0},
			{0, std::cos(twist_rad), -std::sin(twist_rad)},
			{0, std::sin(twist_rad), std::cos(twist_rad)}
		}
	};

	// Apply the twist rotation matrix
	for (int i = 0; i < 3; ++i) {
		direction_vector[i] = twist_rotation[i][0] * direction_vector[0] +
				      twist_rotation[i][1] * direction_vector[1] +
				      twist_rotation[i][2] * direction_vector[2];
	}

	// Normalize and return the direction vector
	return normalize(direction_vector);
}

double ActuatorEffectivenessAvianInspired::computePitchAnge()
{
    // Retrieve vehicle attitude for pitch angle
    vehicle_attitude_s attitude_data;
    if (!_vehicle_attitude_sub.update(&attitude_data)) {
        PX4_ERR("Failed to update attitude data");
        return pitch;
    }

    // Convert quaternion to Euler angles
    matrix::Quatf q(attitude_data.q);
    matrix::Eulerf euler_angles(q);

	pitch = static_cast<double>(euler_angles.theta() * (180.0f / static_cast<float>(M_PI)));

    return pitch;
}

BodyFrameVelocities ActuatorEffectivenessAvianInspired::extractBodyFrameVelocities(double pitch)
{
    BodyFrameVelocities result;
    AirspeedValidator air_speed;

    // Initialize result velocities and validity
    result.vx = last_vel_x;
    result.vy = last_vel_y;
    result.vz = last_vel_z;
    result.valid = false;

    // Retrieve linear acceleration
    sensor_accel_s accel_data;
    if (!_sensor_accel_sub.update(&accel_data)) {
        PX4_ERR("Failed to update linear acceleration data");
        return result;
    }

    // Update velocities directly
    result.vx += static_cast<double>(accel_data.x) * static_cast<double>(DT);
    result.vy += static_cast<double>(accel_data.y) * static_cast<double>(DT);
    result.vz += static_cast<double>(accel_data.z) * static_cast<double>(DT);

    // Apply wind adjustment using pitch
    double wind_adjustment = air_speed.get_aspd_wind_value();
    result.vx += static_cast<double>(cos(pitch)) * wind_adjustment;
    result.vz += static_cast<double>(sin(pitch)) * wind_adjustment;

    // Calculate AoA (only if forward velocity is positive)
    if (result.vx >= 0) {
        result.angle_of_attack = atan2(result.vz, result.vx) * (180.0 / M_PI); // Convert to degrees
    } else {
        result.angle_of_attack = 0; // Null AoA if forward velocity is zero
    }

    // Mark as valid
    result.valid = true;

    // Save updated velocities for next iteration
    last_vel_x = result.vx;
    last_vel_y = result.vy;
    last_vel_z = result.vz;

    return result;
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
