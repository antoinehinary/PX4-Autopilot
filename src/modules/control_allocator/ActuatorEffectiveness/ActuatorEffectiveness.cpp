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

#include "ActuatorEffectiveness.hpp"
#include "../ControlAllocation/ControlAllocation.hpp"

#include <px4_platform_common/log.h>

int ActuatorEffectiveness::Configuration::addActuator(ActuatorType type, const matrix::Vector3f &torque,
		const matrix::Vector3f &thrust)
{
	int actuator_idx = num_actuators_matrix[selected_matrix];

	if (actuator_idx >= NUM_ACTUATORS) {
		PX4_ERR("Too many actuators");
		return -1;
	}

	if ((int)type < (int)ActuatorType::COUNT - 1 && num_actuators[(int)type + 1] > 0) {
		PX4_ERR("Trying to add actuators in the wrong order (add motors first, then servos)");
		return -1;
	}

	effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::ROLL, actuator_idx) = torque(0);
	effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::PITCH, actuator_idx) = torque(1);
	effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::YAW, actuator_idx) = torque(2);
	effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::THRUST_X, actuator_idx) = thrust(0);
	effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::THRUST_Y, actuator_idx) = thrust(1);
	effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::THRUST_Z, actuator_idx) = thrust(2);
	matrix_selection_indexes[totalNumActuators()] = selected_matrix;
	++num_actuators[(int)type];
	return num_actuators_matrix[selected_matrix]++;
}

int ActuatorEffectiveness::Configuration::addActuatoravian(ActuatorType type, const matrix::Vector3f &torque,
        const matrix::Vector3f &thrust)
{
    BodyFrameVelocities vel_body = extractBodyFrameVelocities();
    ServoControl serv_ctrl = getServoControlData();

    // Dummy usage of vel_body and serv_ctrl to suppress warnings
    double dummy_vx = vel_body.vx + vel_body.vy + vel_body.vz;
    bool dummy_valid = vel_body.valid;
    float dummy_control = serv_ctrl.control[0];
    PX4_INFO("Dummy usage: vx=%f, valid=%d, control=%f", dummy_vx, dummy_valid, static_cast<double>(dummy_control));

    int actuator_idx = num_actuators_matrix[selected_matrix];
    // float serv_ctrl_val = serv_ctrl.control[actuator_idx];

    if (actuator_idx >= NUM_ACTUATORS) {
        PX4_ERR("Too many actuators");
        return -1;
    }

    if ((int)type < (int)ActuatorType::COUNT - 1 && num_actuators[(int)type + 1] > 0) {
        PX4_ERR("Trying to add actuators in the wrong order (add motors first, then servos)");
        return -1;
    }

    effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::ROLL, actuator_idx) = torque(0);
    effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::PITCH, actuator_idx) = torque(1);
    effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::YAW, actuator_idx) = torque(2);
    effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::THRUST_X, actuator_idx) = thrust(0);
    effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::THRUST_Y, actuator_idx) = thrust(1);
    effectiveness_matrices[selected_matrix](ControlAllocation::ControlAxis::THRUST_Z, actuator_idx) = thrust(2);
    matrix_selection_indexes[totalNumActuators()] = selected_matrix;
    ++num_actuators[(int)type];
    return num_actuators_matrix[selected_matrix]++;
}

void ActuatorEffectiveness::Configuration::actuatorsAdded(ActuatorType type, int count)
{
	int total_count = totalNumActuators();

	for (int i = 0; i < count; ++i) {
		matrix_selection_indexes[i + total_count] = selected_matrix;
	}

	num_actuators[(int)type] += count;
	num_actuators_matrix[selected_matrix] += count;
}

int ActuatorEffectiveness::Configuration::totalNumActuators() const
{
	int total_count = 0;

	for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
		total_count += num_actuators_matrix[i];
	}

	return total_count;
}

void ActuatorEffectiveness::stopMaskedMotorsWithZeroThrust(uint32_t stoppable_motors_mask, ActuatorVector &actuator_sp)
{
	for (int actuator_idx = 0; actuator_idx < NUM_ACTUATORS; actuator_idx++) {
		const uint32_t motor_mask = (1u << actuator_idx);

		if (stoppable_motors_mask & motor_mask) {

			// Stop motor if its setpoint is below 2%. This value was determined empirically (RC stick inaccuracy)
			if (fabsf(actuator_sp(actuator_idx)) < .02f) {
				_stopped_motors_mask |= motor_mask;

			} else {
				_stopped_motors_mask &= ~motor_mask;
			}
		}
	}
}

/*Helper functions*/
double ActuatorEffectiveness::Configuration::mapRange(double value, double input_min, double input_max,
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


SimpleArray<double, 3> ActuatorEffectiveness::Configuration::flatPlateForce(const SimpleArray<double, 3> &lift_dir,
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

double ActuatorEffectiveness::Configuration::liftCoefficient(double alpha)
{
	double cl_baseline = 0.1;
	return cl_baseline + 2 * std::sin(alpha) * std::cos(alpha);
}

double ActuatorEffectiveness::Configuration::dragCoefficient(double alpha)
{
	double cd_baseline = 0.02;
	return cd_baseline + 2 * std::pow(std::sin(alpha), 2);
}

double ActuatorEffectiveness::Configuration::toRadians(double degrees)
{
	return degrees * M_PI / 180.0;
}

double ActuatorEffectiveness::Configuration::norm(const SimpleArray<double, 3> &vec)
{
	return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::crossProduct(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	return {
		vec1[1] *vec2[2] - vec1[2] *vec2[1],
		vec1[2] *vec2[0] - vec1[0] *vec2[2],
		vec1[0] *vec2[1] - vec1[1] *vec2[0]
	};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::add(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	return {vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2]};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::subtract(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	return {vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2]};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::multiply(const SimpleArray<double, 3> &vec, double scalar)
{
	return {vec[0] *scalar, vec[1] *scalar, vec[2] *scalar};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::normalize(const SimpleArray<double, 3> &vec)
{
	double vec_norm = norm(vec);
	constexpr double epsilon = 1e-9; // Small threshold value to avoid floating-point comparison

	if (std::fabs(vec_norm) < epsilon) {
		return {0.0, 0.0, 0.0};
	}

	return {vec[0] / vec_norm, vec[1] / vec_norm, vec[2] / vec_norm};
}


SimpleArray<double, 3> ActuatorEffectiveness::Configuration::getDirectionVector(double angle_of_attack,
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

BodyFrameVelocities ActuatorEffectiveness::Configuration::extractBodyFrameVelocities()
{
    BodyFrameVelocities result;
    AirspeedValidator air_speed;
    result.vx = 0.0f;
    result.vy = 0.0f;
    result.vz = 0.0f;
    result.valid = false;

    // Retrieve vehicle angular velocity
    vehicle_angular_velocity_s angular_velocity_data;
    if (!_vehicle_angular_velocity_sub.update(&angular_velocity_data)) {
        PX4_ERR("Failed to update angular velocity data");
        return result;
    }

    // Retrieve linear acceleration
    sensor_accel_s accel_data;
    if (!_sensor_accel_sub.update(&accel_data)) {
        PX4_ERR("Failed to update linear acceleration data");
        return result;
    }

    // Retrieve vehicle attitude for pitch angle
    vehicle_attitude_s attitude_data;
    if (!_vehicle_attitude_sub.update(&attitude_data)) {
        PX4_ERR("Failed to update attitude data");
        return result;
    }

    // Convert quaternion to Euler angles
    matrix::Quatf q(attitude_data.q);
    matrix::Eulerf euler_angles(q);

    result.pitch_angle = euler_angles.theta() * (180.0f / static_cast<float>(M_PI));; // Convert pitch (theta) from radians to degrees
    result.pitch_angle_rad = euler_angles.theta();

    // Calculate linear velocities in x and z directions using instantaneous acceleration
    // (This assumes that velocity can be approximated as acceleration * delta_time)
    // Compute the change in velocity
    double delta_vx = static_cast<double>(accel_data.x) * static_cast<double>(DT);
    double delta_vy = static_cast<double>(accel_data.y) * static_cast<double>(DT);
    double delta_vz = static_cast<double>(accel_data.z) * static_cast<double>(DT);

    result.vx = last_vel_x + delta_vx; // Approximated or instantaneous velocity in the x-axis (forward)
    result.vy = last_vel_y + delta_vy; // Approximated or instantaneous velocity in the x-axis (forward)
    result.vz = last_vel_z + delta_vz; // Approximated or instantaneous velocity in the z-axis (downward)
    result.valid = true;

    // Update last velocities for the next call
    last_vel_x = result.vx;
    last_vel_y = result.vy;
    last_vel_z = result.vz;

    // Update velocities with explicit casting to double for result.vx and result.vz
    result.vx = result.vx + static_cast<double>(cos(result.pitch_angle_rad) * air_speed.get_aspd_wind_value());
    result.vz = result.vz + static_cast<double>(sin(result.pitch_angle_rad) * air_speed.get_aspd_wind_value());

    // Calculate AoA using linear velocity in body frame
    if (result.vx >= 0) {
        result.angle_of_attack = atan2(result.vz, result.vx) * static_cast<double>(180.0f / static_cast<float>(M_PI)); // Convert to degrees
    } else {
        result.angle_of_attack = 0; // Null AoA if forward velocity is zero
    }

    return result;
}


// Function to update the ServoControl struct with the latest data from the topic
ServoControl ActuatorEffectiveness::Configuration::getServoControlData()
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
