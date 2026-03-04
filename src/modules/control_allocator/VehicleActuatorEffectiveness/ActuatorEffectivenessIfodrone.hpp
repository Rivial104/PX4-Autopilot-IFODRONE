/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IFODRONE:
 *
 *   Motors 0-1 : coaxial pair, upward axis (Z-thrust + yaw differential).
 *   Motors 2-5 : side EDFs, tilted by servos.  Axes re-linearized every
 *                iteration from tilt angles.  Idle trim keeps them spinning.
 *                Natural roll/pitch/Z effectiveness from tilted axes.
 *   Tilts  0-3 : servo actuators allocated by CA for roll/pitch.
 *                Effectiveness = 15.0 (high value to overcome normalization).
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

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	const char *name() const override { return "IfoDrone"; }

private:
	float neutralTiltControl(int tilt_index) const;
	void updateTiltOffsets();
	void overwriteTiltEffectiveness(EffectivenessMatrix &effectiveness) const;

	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;

	ActuatorVector _tilt_offsets;
	ActuatorVector _last_actuator_sp;
	int _first_tilt_idx{-1};
	bool _tilt_setpoint_updated{true};
	bool _tilt_setpoints_initialized{false};

	// --- Tuning constants ---
	static constexpr float _side_motor_idle = 0.05f;  ///< idle trim for side motors [0..1]
};
