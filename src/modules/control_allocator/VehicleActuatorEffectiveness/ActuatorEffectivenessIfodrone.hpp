/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IfoDrone.
 *
 * Both motors and tilt servos are allocated by the CA.
 * ifo_att_control publishes vehicle_torque_setpoint (roll + pitch + yaw);
 * the CA translates that to motor speeds and tilt angles.
 *
 *   Motors 0-1 : coaxial pair, fixed −Z axis (Z-thrust + yaw via KM)
 *   Motors 2-5 : side EDFs, axes tilted up by the hover angle (lift + lateral thrust)
 *   Servos 0-3 : tilt servos for roll/pitch (front, right, back, left), biased to hover tilt
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

protected:
	// Nominal tilt angle of the side EDFs (degrees up from horizontal). 0 ⇒ the EDFs sit
	// level/horizontal with zero roll/pitch demand (pure XY lateral thrust); the airframe's
	// EDF thrust axes (CA_ROTOR2..5) are horizontal (radial, Z=0) to match. Lift is carried
	// by the coaxial pair; the servos tilt the EDFs off-level only to command roll/pitch.
	static constexpr float HOVER_TILT_DEG{0.f};

	ActuatorEffectivenessRotors _mc_motors;
	ActuatorEffectivenessTilts  _tilts;

	int            _first_tilt_idx{0};
	ActuatorVector _tilt_offsets{};
};
