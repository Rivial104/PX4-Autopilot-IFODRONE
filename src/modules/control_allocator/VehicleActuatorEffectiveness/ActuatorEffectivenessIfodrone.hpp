/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IfoDrone.
 *
 * Only motors are allocated here.
 * Tilt servo angles are commanded directly by ifo_att_control
 * and read back here to keep motor axes up-to-date.
 *
 *   Motors 0-1 : coaxial pair, fixed −Z axis (Z-thrust + yaw via KM)
 *   Motors 2-5 : side EDFs, axes updated dynamically from tilt state
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
	ActuatorEffectivenessTilts  _tilts;

	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	ActuatorVector     _current_tilt_values;
};
