uint64 timestamp # time since system start (microseconds)

float32 collective_tilt_normalized_setpoint	# Collective tilt angle of motors of tiltrotor, 0: vertical, 1: horizontal [0, 1]
float32 collective_thrust_normalized_setpoint 	# Collective thrust setpoint [0, 1]

# IFODRONE individual tilt angles (radians)
# Used for 4-tilt configuration where tilts can be controlled independently
# Assumption: tilt_front = -tilt_back, tilt_right = -tilt_left for zero Z force
float32 tilt_front	# Front motor tilt angle (positive = tilt forward)
float32 tilt_right	# Right motor tilt angle (positive = tilt right)
float32 tilt_back	# Back motor tilt angle (should be -tilt_front)
float32 tilt_left	# Left motor tilt angle (should be -tilt_right)
