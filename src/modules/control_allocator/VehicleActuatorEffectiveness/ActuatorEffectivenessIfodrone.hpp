/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IfoDrone.
 *
 * The full 6-DOF wrench setpoint [torque; thrust] is allocated to all 10
 * actuators at once; the matrix is re-linearized every cycle around the last
 * actuator setpoint (EDF axes rotated by the current tilt angles, tilt-servo
 * columns scaled by the current EDF thrust).
 *
 *   Motors 0-1 : coaxial pair, fixed −Z axis (Z-thrust + yaw via KM)
 *   Motors 2-5 : side EDFs, axes tilted by the current servo setpoints
 *   Servos 0-3 : tilt servos (front, right, back, left) — full wrench Jacobian
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

	// No RPY normalization: it would cancel the thrust-dependent scaling of the
	// tilt-servo columns (tilt torque authority is proportional to EDF thrust).
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = false;
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	const char *name() const override { return "IfoDrone"; }

protected:
	float tiltTrim(int tilt_index, float hover_angle_rad) const;

	ActuatorEffectivenessRotors _mc_motors;
	ActuatorEffectivenessTilts  _tilts;

	int            _first_tilt_idx{0};
	ActuatorVector _last_actuator_sp{};   ///< linearization point (last allocation result)
	bool           _last_actuator_sp_valid{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_TILT_HOVER>) _param_ifo_tilt_hover,
		(ParamFloat<px4::params::IFO_EDF_TRIM>)   _param_ifo_edf_trim
	)
};
