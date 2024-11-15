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


/* Comments to help
 *  get the wind value param :
	#include "AirspeedValidator.hpp"
	int32 aspd_wind = get_aspd_wind_value();

 *
 *
 *
 *
 */



#include "ActuatorEffectiveness.hpp"
#include "../ControlAllocation/ControlAllocation.hpp"

#include <px4_platform_common/log.h>

// constexpr double AIR_DENSITY = 1.225;         // kg/m^3, standard air density at sea level (ISA conditions)
// constexpr double TAIL_LENGTH = 0.22;          // in meters
// constexpr double TAIL_INITIAL_WIDTH = 0.07;   // in meters
// constexpr double MIN_TAIL_TWIST_ANGLE = -20.0; // in degrees
// constexpr double MAX_TAIL_TWIST_ANGLE = 20.0;  // in degrees
// // constexpr double MIN_TAIL_AREA = 0.0584; // in m^2 // redefined in .hpp
// // constexpr double MAX_TAIL_AREA = 0.461;  // in m^2 // redefined in .hpp
// constexpr double MIN_TAIL_PITCH_ANGLE = -30.0; // in degrees
// constexpr double MAX_TAIL_PITCH_ANGLE = 30.0;  // in degrees
// constexpr double MIN_TAIL_EXPANSION_ANGLE = -60.0; // in degrees
// constexpr double MAX_TAIL_EXPANSION_ANGLE = 60.0;  // in degrees

// constexpr double MAX_TAIL_AREA = 0.461;  // in m^2

// // Wing
// constexpr double WING_SPAN = 1.0;             // in meters
// constexpr double BODY_LENGTH = 0.5;	      // in meters
// constexpr double MIN_WING_LENGTH = 0.38;      // in meters
// constexpr double DT = 0.01;      // in meters



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

int ActuatorEffectiveness::Configuration::addActuatoravian(ActuatorType type, matrix::Vector3f &torque,
	matrix::Vector3f &thrust)
{
	int actuator_idx = num_actuators_matrix[selected_matrix];

	ServoControl serv_ctrl = getServoControlData();
	BodyFrameVelocities body_vel = extractBodyFrameVelocities();

	double serv_ctrl_val = serv_ctrl.control[actuator_idx];

	if (actuator_idx >= NUM_ACTUATORS) {
		PX4_ERR("Too many actuators");
		return -1;
	}

	if ((int)type < (int)ActuatorType::COUNT - 1 && num_actuators[(int)type + 1] > 0) {
		PX4_ERR("Trying to add actuators in the wrong order (add motors first, then servos)");
		return -1;
	}

	float angle_of_attack = body_vel.angle_of_attack;
	float vx = body_vel.vx;
	float vy = body_vel.vy;
	float vz = body_vel.vz;

	switch (actuator_idx) {
	case 1: // Left wing
	case 2: { // Right wing
			wing_length = mapRange(serv_ctrl_val, -1.0, 1.0, MIN_WING_LENGTH, WING_SPAN);
			wing_area = wing_length * WING_WIDTH;

			SimpleArray<double, 3> direction = getDirectionVector(angle_of_attack, 0.0);
			SimpleArray<double, 3> velocity = {vx, vy, vz};
			SimpleArray<double, 3> force = flatPlateForce(direction, velocity, wing_area, angle_of_attack);

			thrust = {static_cast<float>(force[0]), static_cast<float>(force[1]), static_cast<float>(force[2])};
			wing_offset = (actuator_idx == 1) ? -wing_length / 2 : wing_length / 2;
			matrix::Vector3f moment_arm(0.0f, wing_offset, 0.0f);
			torque = moment_arm.cross(matrix::Vector3f(force[0], force[1], force[2]));

			break;
		}

	case 3: { // Tail pitch
			tail_pitch_angle = mapRange(serv_ctrl_val, -1.0, 1.0, MIN_TAIL_PITCH_ANGLE, MAX_TAIL_PITCH_ANGLE);

			SimpleArray<double, 3> direction = getDirectionVector(tail_pitch_angle, 0.0);
			SimpleArray<double, 3> velocity = {vx, vy, vz};
			SimpleArray<double, 3> force = flatPlateForce(direction, velocity, tail_area, tail_pitch_angle);

			thrust = {static_cast<float>(force[0]), static_cast<float>(force[1]), static_cast<float>(force[2])};
			matrix::Vector3f moment_arm(-(BODY_LENGTH + TAIL_LENGTH) / 2, 0.0f, 0.0f);
			torque = moment_arm.cross(matrix::Vector3f(force[0], force[1], force[2]));

			break;
		}

	case 4: { // Tail twist
			tail_twist_angle = mapRange(serv_ctrl_val, -1.0, 1.0, MIN_TAIL_TWIST_ANGLE, MAX_TAIL_TWIST_ANGLE);

			SimpleArray<double, 3> direction = getDirectionVector(angle_of_attack, tail_twist_angle);
			SimpleArray<double, 3> velocity = {vx, vy, vz};
			SimpleArray<double, 3> force = flatPlateForce(direction, velocity, tail_area, angle_of_attack);

			thrust = {static_cast<float>(force[0]), static_cast<float>(force[1]), static_cast<float>(force[2])};
			matrix::Vector3f moment_arm(-(BODY_LENGTH + TAIL_LENGTH) / 2, 0.0f, 0.0f);
			torque = moment_arm.cross(matrix::Vector3f(force[0], force[1], force[2]));

			break;
		}

	case 5: { // Tail area
			tail_expansion_angle = mapRange(serv_ctrl_val, -1.0, 1.0, MIN_TAIL_EXPANSION_ANGLE, MAX_TAIL_EXPANSION_ANGLE);
			tail_area = (abs(tail_expansion_angle) / 360.0) * M_PI * TAIL_LENGTH * (TAIL_INITIAL_WIDTH / 2.0);

			SimpleArray<double, 3> direction = getDirectionVector(angle_of_attack, 0.0);
			SimpleArray<double, 3> velocity = {vx, vy, vz};
			SimpleArray<double, 3> force = flatPlateForce(direction, velocity, tail_area, angle_of_attack);

			thrust = {static_cast<float>(force[0]), static_cast<float>(force[1]), static_cast<float>(force[2])};
			matrix::Vector3f moment_arm(-(BODY_LENGTH + TAIL_LENGTH) / 2, 0.0f, 0.0f);
			torque = moment_arm.cross(matrix::Vector3f(force[0], force[1], force[2]));

			break;
		}

	default:
		PX4_ERR("Invalid actuator index");
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

/*Helper Functoins*/

double ActuatorEffectiveness::Configuration::mapRange(double value, double input_min, double input_max,
		double output_min, double output_max)
{
	constexpr double epsilon = 1e-6; // Threshold to avoid division by zero

	if (fabs(input_max - input_min) < epsilon) {
		PX4_ERR("Invalid input range");
		return output_min; // Fallback value
	}

	// Linear interpolation
	double scaled_value = (value - input_min) / (input_max - input_min);
	return output_min + scaled_value * (output_max - output_min);
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::flatPlateForce(
		const SimpleArray<double, 3> &lift_dir, const SimpleArray<double, 3> &velocity,
		double surface_area, double alpha)
{
	constexpr double epsilon = 1e-6; // Threshold to avoid zero-force computations

	if (fabs(surface_area) < epsilon || norm(velocity) < epsilon) {
		return {0.0, 0.0, 0.0};
	}

	double dynamic_pressure = 0.5 * AIR_DENSITY * std::pow(norm(velocity), 2);
	double cl = liftCoefficient(toRadians(alpha));
	double cd = dragCoefficient(toRadians(alpha));

	// Lift force is perpendicular to velocity
	auto lift_force = multiply(lift_dir, dynamic_pressure * surface_area * cl);

	// Drag force is along the velocity direction
	auto drag_force = multiply(normalize(velocity), dynamic_pressure * surface_area * cd);

	// Total force is the vector sum of lift and drag
	return add(lift_force, drag_force);
}

double ActuatorEffectiveness::Configuration::liftCoefficient(double alpha)
{
	return 0.1 + 2 * std::sin(alpha) * std::cos(alpha); // Simplified lift coefficient model
}

double ActuatorEffectiveness::Configuration::dragCoefficient(double alpha)
{
	return 0.02 + 2 * std::pow(std::sin(alpha), 2); // Simplified drag coefficient model
}

double ActuatorEffectiveness::Configuration::toRadians(double degrees)
{
	return degrees * M_PI / 180.0; // Degrees to radians conversion
}

double ActuatorEffectiveness::Configuration::norm(const SimpleArray<double, 3> &vec)
{
	// Compute Euclidean norm
	return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::normalize(const SimpleArray<double, 3> &vec)
{
	constexpr double epsilon = 1e-9; // Small value to avoid division by zero
	double vec_norm = norm(vec);

	if (fabs(vec_norm) < epsilon) {
		return {0.0, 0.0, 0.0}; // Return zero vector for invalid normalization
	}

	return {vec[0] / vec_norm, vec[1] / vec_norm, vec[2] / vec_norm};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::add(const SimpleArray<double, 3> &vec1,
		const SimpleArray<double, 3> &vec2)
{
	// Vector addition
	return {vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2]};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::multiply(const SimpleArray<double, 3> &vec, double scalar)
{
	// Scalar multiplication
	return {vec[0] * scalar, vec[1] * scalar, vec[2] * scalar};
}

SimpleArray<double, 3> ActuatorEffectiveness::Configuration::getDirectionVector(double angle_of_attack,
		double twist_angle)
{
	// Convert angles to radians
	double aoa_rad = toRadians(angle_of_attack);
	double twist_rad = toRadians(twist_angle);

	// Base direction vector from the angle of attack
	SimpleArray<double, 3> direction_vector = {std::sin(aoa_rad), 0.0, -std::cos(aoa_rad)};

	// Twist rotation matrix (rotation around the x-axis)
	SimpleArray<SimpleArray<double, 3>, 3> twist_rotation = {{
			{1, 0, 0},
			{0, std::cos(twist_rad), -std::sin(twist_rad)},
			{0, std::sin(twist_rad), std::cos(twist_rad)}
		}};

	// Apply rotation matrix
	SimpleArray<double, 3> rotated_vector = {0.0, 0.0, 0.0};

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rotated_vector[i] += twist_rotation[i][j] * direction_vector[j];
		}
	}

	return normalize(rotated_vector); // Ensure the result is normalized
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

BodyFrameVelocities ActuatorEffectiveness::Configuration::extractBodyFrameVelocities()
{
    BodyFrameVelocities result;
    AirspeedValidator airspeed_validator;

    // Get wind speed from AirspeedValidator
    int32_t wind_speed = airspeed_validator.get_aspd_wind_value();

    result.vx = 0.0f;
    result.vy = 0.0f;
    result.vz = 0.0f;
    result.valid = false;

    vehicle_odometry_s odometry_data;
    if (_vehicle_odometry_sub.update(&odometry_data)) {
        if (odometry_data.velocity_frame != vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD) {
            PX4_ERR("Velocity frame mismatch. Expected BODY_FRD");
            return result;
        }
        } else {
            PX4_ERR("Failed to update vehicle odometry");
        return result;
    }


    // Check if velocity is in body frame
    if (odometry_data.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD) {
        // Velocity is already in body frame
        result.vx = odometry_data.velocity[0];
        result.vy = odometry_data.velocity[1];
        result.vz = odometry_data.velocity[2];
    } else {
        PX4_ERR("Velocity frame is not BODY_FRD. Current frame: %d", odometry_data.velocity_frame);
        return result; // Exit early if frame is incorrect
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

    result.pitch_angle = euler_angles.theta() * (180.0f / static_cast<float>(M_PI)); // Convert pitch (theta) from radians to degrees
    result.pitch_angle_rad = euler_angles.theta();

    // Update velocities using the wind value
    result.vx += static_cast<double>(cos(result.pitch_angle_rad) * wind_speed);
    result.vz += static_cast<double>(sin(result.pitch_angle_rad) * wind_speed);

    // Calculate AoA using linear velocity in body frame
    if (result.vx >= 0) {
        result.angle_of_attack = atan2(result.vz, result.vx) * static_cast<double>(180.0f / static_cast<float>(M_PI)); // Convert to degrees
    } else {
        result.angle_of_attack = 0; // Null AoA if forward velocity is zero
    }

    result.valid = true;
    return result;
}
