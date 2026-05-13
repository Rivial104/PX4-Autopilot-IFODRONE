/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IfoDrone configuration.
 * Motor forces are allocated by CA. Tilt servos are commanded externally
 * by the attitude controller (ifo_att_control), but CA subscribes to their
 * positions to update motor axes dynamically.
 *
 *   Motors 0-1 : coaxial pair, upward axis (Z-thrust + yaw via differential thrust)
 *   Motors 2-5 : side EDFs with dynamic axes based on tilt positions
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>

class ActuatorEffectivenessIfodrone : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessIfodrone(ModuleParams *parent);
	virtual ~ActuatorEffectivenessIfodrone() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

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
	ActuatorEffectivenessRotors _mc_motors;
	ActuatorEffectivenessTilts _tilts;  // Used to read tilt geometry; NOT added as CA actuators

	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	ActuatorVector _current_tilt_values;
};
