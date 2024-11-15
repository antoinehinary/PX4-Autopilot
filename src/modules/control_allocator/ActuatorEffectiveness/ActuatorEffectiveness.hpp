/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 * @file ActuatorEffectiveness.hpp
 *
 * Interface for Actuator Effectiveness
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <cstdint>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_attitude.h>
#include "/home/antoine/PX4-Autopilot/src/modules/airspeed_selector/AirspeedValidator.hpp"

#include <uORB/uORB.h>
#include <parameters/param.h>
#include <cmath>

#include <matrix/math.hpp>
#include <math.h>


constexpr double AIR_DENSITY = 1.225;         // kg/m^3, standard air density at sea level (ISA conditions)
constexpr double TAIL_LENGTH = 0.22;          // in meters
constexpr double TAIL_INITIAL_WIDTH = 0.07;   // in meters
constexpr double MIN_TAIL_TWIST_ANGLE = -20.0; // in degrees
constexpr double MAX_TAIL_TWIST_ANGLE = 20.0;  // in degrees
constexpr double MIN_TAIL_AREA = 0.0584; // in m^2 // redefined in .hpp
constexpr double MAX_TAIL_AREA = 0.461;  // in m^2 // redefined in .hpp
constexpr double MIN_TAIL_PITCH_ANGLE = -30.0; // in degrees
constexpr double MAX_TAIL_PITCH_ANGLE = 30.0;  // in degrees
constexpr double MIN_TAIL_EXPANSION_ANGLE = -60.0; // in degrees
constexpr double MAX_TAIL_EXPANSION_ANGLE = 60.0;  // in degrees
constexpr double MAX_WING_LENGTH = 0.5;       // in meters

// Wing
constexpr double WING_SPAN = 1.0;             // in meters
constexpr double WING_WIDTH = 0.3;           // in meters
constexpr double BODY_LENGTH = 0.5;	      // in meters
constexpr double MIN_WING_LENGTH = 0.38;      // in meters
constexpr double DT = 0.01;      // in meters

enum class AllocationMethod {
	NONE = -1,
	PSEUDO_INVERSE = 0,
	SEQUENTIAL_DESATURATION = 1,
	AUTO = 2,
};

enum class ActuatorType {
	MOTORS = 0,
	SERVOS,

	COUNT
};

enum class EffectivenessUpdateReason {
	NO_EXTERNAL_UPDATE = 0,
	CONFIGURATION_UPDATE = 1,
	MOTOR_ACTIVATION_UPDATE = 2,
};

template <typename T, size_t N>
struct SimpleArray {
    T data[N];

    // Access operator
    T& operator[](size_t index) {
        return data[index];
    }

    // Const access operator
    const T& operator[](size_t index) const {
        return data[index];
    }

    // Fill method
    void fill(const T& value) {
        for (size_t i = 0; i < N; ++i) {
            data[i] = value;
        }
    }

    // Size method
    constexpr size_t size() const {
        return N;
    }
};

// Define the ServoControl struct
struct ServoControl
{
	uint64_t timestamp;         // time since system start (microseconds)
	uint64_t timestamp_sample;  // timestamp the data this control response is based on was sampled

	static constexpr uint8_t NUM_CONTROLS = 8;
	SimpleArray<float, NUM_CONTROLS> control; // array to hold control values ranging from [-1, 1]

	// Constructor to initialize values
	ServoControl()
		: timestamp(0), timestamp_sample(0)
	{
		control.fill(NAN); // Initialize all controls to NaN by default to represent disarmed state
	}

	// bool to check for validity
	bool valid;
};

struct BodyFrameVelocities {
	double vx; // in m/s
	double vz; // in m/s
	double vy; // in m/s
	double pitch_angle; // in degrees
	double pitch_angle_rad; // in radians
	double angle_of_attack; // in degrees
	bool valid;
};

class ActuatorEffectiveness
{
public:
	ActuatorEffectiveness() = default;
	virtual ~ActuatorEffectiveness() = default;

	static constexpr int NUM_ACTUATORS = 16;
	static constexpr int NUM_AXES = 6;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};

	static constexpr int MAX_NUM_MATRICES = 2;

	using EffectivenessMatrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;
	using ActuatorVector = matrix::Vector<float, NUM_ACTUATORS>;

	enum class FlightPhase {
		HOVER_FLIGHT = 0,
		FORWARD_FLIGHT = 1,
		TRANSITION_HF_TO_FF = 2,
		TRANSITION_FF_TO_HF = 3
	};

	struct Configuration {

		/**
		 * Add an actuator to the selected matrix, returning the index, or -1 on error
		 */
		int addActuator(ActuatorType type, const matrix::Vector3f &torque, const matrix::Vector3f &thrust);

		/**
		 * Add an actuator for avian inspired to the selected matrix, returning the index, or -1 on error
		 */
		int addActuatoravian(ActuatorType type, matrix::Vector3f &torque, matrix::Vector3f &thrust);

		/**
		 * Call this after manually adding N actuators to the selected matrix
		 */
		void actuatorsAdded(ActuatorType type, int count);

		int totalNumActuators() const;

		/// Configured effectiveness matrix. Actuators are expected to be filled in order, motors first, then servos
		EffectivenessMatrix effectiveness_matrices[MAX_NUM_MATRICES];

		int num_actuators_matrix[MAX_NUM_MATRICES]; ///< current amount, and next actuator index to fill in to effectiveness_matrices
		ActuatorVector trim[MAX_NUM_MATRICES];

		ActuatorVector linearization_point[MAX_NUM_MATRICES];

		int selected_matrix;

		uint8_t matrix_selection_indexes[NUM_ACTUATORS * MAX_NUM_MATRICES];
		int num_actuators[(int)ActuatorType::COUNT];

		/**
		 * Maps a value from one range to another.
		 */
		double mapRange(double value, double input_min, double input_max, double output_min, double output_max);

		/**
		 * Computes aerodynamic force using a flat-plate model.
		 */
		SimpleArray<double, 3> flatPlateForce(const SimpleArray<double, 3> &lift_dir,
			const SimpleArray<double, 3> &velocity, double surface_area, double alpha);

		/**
		 * Calculates the lift coefficient based on angle of attack.
		 */
		double liftCoefficient(double alpha);

		/**
		 * Calculates the drag coefficient based on angle of attack.
		 */
		double dragCoefficient(double alpha);

		/**
		 * Converts degrees to radians.
		 */
		double toRadians(double degrees);

		/**
		 * Calculates the Euclidean norm (magnitude) of a 3D vector.
		 */
		double norm(const SimpleArray<double, 3> &vec);

		/**
		 * Normalizes a 3D vector to unit length.
		 */
		SimpleArray<double, 3> normalize(const SimpleArray<double, 3> &vec);

		/**
		 * Performs vector addition for two 3D vectors.
		 */
		SimpleArray<double, 3> add(const SimpleArray<double, 3> &vec1, const SimpleArray<double, 3> &vec2);

		/**
		 * Scales a 3D vector by a scalar value.
		 */
		SimpleArray<double, 3> multiply(const SimpleArray<double, 3> &vec, double scalar);

		/**
		 * Computes the aerodynamic direction vector.
		 */
		SimpleArray<double, 3> getDirectionVector(double angle_of_attack, double twist_angle);

		/**
		 * Extracts the servo control input.
		 */
		ServoControl getServoControlData();

		/**
		 * Extracts the body velocities and needed angles.
		 */
		BodyFrameVelocities extractBodyFrameVelocities();

		/**
		 * Subscriptions to required topics.
		 * These provide the body velocities and servo control values.
		 */
		uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
		uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
		uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
		uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
		uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};


		double tail_area = MAX_TAIL_AREA;
		double wing_length = MAX_WING_LENGTH;       // Default to max wing length
		double wing_area = wing_length * WING_WIDTH;
		double wing_offset = 0.0;
		double tail_pitch_angle = 0.0;
		double tail_twist_angle = 0.0;
		double tail_expansion_angle = 0.0;

	};

	/**
	 * Set the current flight phase
	 *
	 * @param Flight phase
	 */
	virtual void setFlightPhase(const FlightPhase &flight_phase)
	{
		_flight_phase = flight_phase;
	}

	/**
	 * Get the number of effectiveness matrices. Must be <= MAX_NUM_MATRICES.
	 * This is expected to stay constant.
	 */
	virtual int numMatrices() const { return 1; }

	/**
	 * Get the desired allocation method(s) for each matrix, if configured as AUTO
	 */
	virtual void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const
	{
		for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
			allocation_method_out[i] = AllocationMethod::PSEUDO_INVERSE;
		}
	}

	/**
	 * Query if the roll, pitch and yaw columns of the mixing matrix should be normalized
	 */
	virtual void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const
	{
		for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
			normalize[i] = false;
		}
	}

	/**
	 * Get the control effectiveness matrix if updated
	 *
	 * @return true if updated and matrix is set
	 */
	virtual bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) { return false;}

	/**
	 * Get the current flight phase
	 *
	 * @return Flight phase
	 */
	const FlightPhase &getFlightPhase() const
	{
		return _flight_phase;
	}

	/**
	 * Display name
	 */
	virtual const char *name() const = 0;

	/**
	 * Callback from the control allocation, allowing to manipulate the setpoint.
	 * Used to allocate auxiliary controls to actuators (e.g. flaps and spoilers).
	 *
	 * @param actuator_sp input & output setpoint
	 */
	virtual void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) {}

	/**
	 * Callback from the control allocation, allowing to manipulate the setpoint.
	 * This can be used to e.g. add non-linear or external terms.
	 * It is called after the matrix multiplication and before final clipping.
	 * @param actuator_sp input & output setpoint
	 */
	virtual void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
				    int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
				    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) {}

	/**
	 * Get a bitmask of motors to be stopped
	 */
	virtual uint32_t getStoppedMotors() const { return _stopped_motors_mask; }

	/**
	 * Fill in the unallocated torque and thrust, customized by effectiveness type.
	 * Can be implemented for every type separately. If not implemented then the effectivenes matrix is used instead.
	 */
	virtual void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) {}

	/**
	 * Stops motors which are masked by stoppable_motors_mask and whose demanded thrust is zero
	 *
	 * @param stoppable_motors_mask mask of motors that should be stopped if there's no thrust demand
	 * @param actuator_sp outcome of the allocation to determine if the motor should be stopped
	 */
	virtual void stopMaskedMotorsWithZeroThrust(uint32_t stoppable_motors_mask, ActuatorVector &actuator_sp);

protected:
	FlightPhase _flight_phase{FlightPhase::HOVER_FLIGHT};
	uint32_t _stopped_motors_mask{0};


};
