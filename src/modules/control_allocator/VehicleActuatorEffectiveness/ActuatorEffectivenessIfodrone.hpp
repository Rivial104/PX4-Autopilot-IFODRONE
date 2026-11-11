/**
 * @file ActuatorEffectivenessIfodrone.hpp
 *
 * Actuator effectiveness for IfoDrone.
 *
 * The full 6-DOF wrench setpoint [torque; thrust] is allocated to all 10
 * actuators at once; the matrix is re-linearized every cycle around the
 * actuator operating point (EDF axes rotated by the current tilt angles,
 * tilt-servo columns scaled by the current EDF thrust). The operating point is
 * the clipped allocation result, low-pass filtered with IFO_CA_LIN_TAU: feeding
 * the raw previous setpoint straight back into the matrix is a discrete-time
 * algebraic loop with a negative eigenvalue (servo setpoints alternate from one
 * allocation cycle to the next, and diverge into a full-range limit cycle at
 * large hover tilt angles).
 *
 *   Motors 0-1 : coaxial pair, fixed −Z axis (Z-thrust + yaw via KM)
 *   Motors 2-5 : side EDFs, axes tilted by the current servo setpoints
 *   Motor  6   : optional vertical booster (IFO_BST_ROTOR): 4 upward EDFs on one
 *                output, slaved to the coax mean command (IFO_BST_RATIO); its
 *                column is folded into the coax columns, never allocated itself
 *   Servos 0-3 : tilt servos (front, right, back, left) — torque-only Jacobian
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <drivers/drv_hrt.h>

class ActuatorEffectivenessIfodrone : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessIfodrone(ModuleParams *parent);
	virtual ~ActuatorEffectivenessIfodrone() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	// Pseudo-inverse + clipping. Sequential desaturation must NOT be used: its
	// mixYaw() step reduces thrust to make room for yaw, and on IFODRONE the
	// coaxial pair is both the only Fz source and the only yaw source, so a yaw
	// error silently eats the collective thrust. Returning PSEUDO_INVERSE here
	// makes CA_METHOD = 2 (Automatic) safe as well, not only CA_METHOD = 0.
	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::PSEUDO_INVERSE;
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
	float hoverAngleRad() const;
	float edfTrim() const;                       ///< IFO_EDF_TRIM clamped to [0, 0.9]
	bool  isSideEdf(int rotor_index) const;      ///< tilting side EDF (vs fixed coax motor)
	int   rotorForTilt(int tilt_index, int num_rotors) const;  ///< rotor driven by this tilt servo, or -1
	void  ensureLinPoint(int num_rotors);        ///< initialize _lin_point to hover on first use
	int   boosterIndex(int num_rotors);          ///< validated IFO_BST_ROTOR, or -1 (booster disabled)
	float boosterRatio() const;                  ///< IFO_BST_RATIO clamped to [0, 10]

	ActuatorEffectivenessRotors _mc_motors;
	ActuatorEffectivenessTilts  _tilts;

	/// Hover operating point: EDFs at IFO_EDF_TRIM, tilts at IFO_TILT_HOVER
	ActuatorVector hoverOperatingPoint(int num_rotors) const;

	/// Lower bound of the EDF thrust used for the roll/pitch row unit (torque_ref)
	static constexpr float EDF_TRIM_REF_MIN{0.05f};

	/// Coaxial pair the booster is slaved to (always CA rotors 0 and 1)
	static constexpr int COAX_TOP{0};
	static constexpr int COAX_BOTTOM{1};

	int            _first_tilt_idx{0};
	ActuatorVector _lin_point{};          ///< linearization point (filtered, clipped allocation result)
	bool           _lin_point_valid{false};
	hrt_abstime    _lin_point_last_update{0};
	int            _booster_idx{-1};          ///< booster actuator index used by updateSetpoint(), -1 = none
	int32_t        _booster_warned_param{-1}; ///< IFO_BST_ROTOR value already warned about (warn once per value)

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_TILT_HOVER>) _param_ifo_tilt_hover,
		(ParamFloat<px4::params::IFO_EDF_TRIM>)   _param_ifo_edf_trim,
		(ParamFloat<px4::params::IFO_EDF_GAIN>)   _param_ifo_edf_gain,
		(ParamFloat<px4::params::IFO_TILT_GAIN>)  _param_ifo_tilt_gain,
		(ParamFloat<px4::params::IFO_CA_LIN_TAU>) _param_ifo_ca_lin_tau,
		(ParamInt<px4::params::IFO_BST_ROTOR>)    _param_ifo_bst_rotor,
		(ParamFloat<px4::params::IFO_BST_RATIO>)  _param_ifo_bst_ratio
	)
};
