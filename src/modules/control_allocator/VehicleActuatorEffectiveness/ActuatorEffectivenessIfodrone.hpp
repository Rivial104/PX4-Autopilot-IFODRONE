/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IfoDrone configuration:
 * - 2 main motors (indices 0, 1) oriented in Z-axis for vertical thrust
 *   - Yaw control via differential thrust (CW/CCW)
 * - 4 side motors (indices 2, 3, 4, 5) in plus configuration with tilt capability
 *   - Roll control via right/left motor tilts (motors 3, 5)
 *   - Pitch control via front/back motor tilts (motors 2, 4)
 *
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

class ActuatorEffectivenessIfodrone : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessIfodrone(ModuleParams *parent);
	virtual ~ActuatorEffectivenessIfodrone() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	const char *name() const override { return "IfoDrone"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:
	ActuatorVector _tilt_offsets;
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;

	// Motor indices
	int _first_main_idx{0};                        ///< Index of first main motor (Z-axis)
	int _first_side_idx{MAIN_MOTORS_NUM};          ///< Index of first side motor (tilting)
	int _first_tilt_idx{-1};                       ///< Index of first tilt servo

	// Motor counts
	static constexpr int MAIN_MOTORS_NUM{2};       ///< Number of main Z-axis motors
	static constexpr int SIDE_MOTORS_NUM{4};       ///< Number of side tilting motors
	static constexpr int MOTORS_NUM{MAIN_MOTORS_NUM + SIDE_MOTORS_NUM};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos{false};
		bool tilt_yaw_neg{false};
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};
};
