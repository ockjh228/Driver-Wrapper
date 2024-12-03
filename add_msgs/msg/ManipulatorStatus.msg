# ManipulatorStatus.msg
# 2024.11.06

# 0: unspecified
# 1: base initilaliztion
# 2: idle
# 3: initilazation
# 4: in fault (red)
# 5: manintenance
# 6: servoing low level
# 7: servoing ready (green)
# 8: servoing playing sequence
# 9: servoing manually controlled
# 255: reserved
int16 arm_state

# 0: unspecified operating mode
# 1: manintenance mode
# 2: update mode
# 3: update completed mode
# 4: update failed mode
# 5: shutting down mode
# 6: run mode
# 7: updating device mode
int8 operating_mode

# 0: unspecified servoing mode
# 2: single level servoing
# 3: low level servoing
# 4: bypass servoing
int8 servoing_mode

# Current joint angles of the robot's actuators
float64[] joint_angles

# Current position of the robot's gripper (0~1) (1:close ~ 0:open)
float64 gripper_position

# Current cartesian position of the end-effector(x,y,z,roll,pitch,yaw)
geometry_msgs/Pose end_effector_position

# Force and torque applied at the end-effector (N*m)
float32[] end_effector_force

#Linear and Angular velocity of the end-effector (m/s ,rad/s)
float32[] velocity

# 0: unspecified error type
# 1: the desired pose is outside robot workspace
# 2: the provided number of joint values does not match robot's number of actuators
# 3: the duration exceeds a limit or is invalid
# 4: the robot's actuators are already at the targeted configuration
# 5: the joint speed exceeds a limit, initial speed does not match current robot speed or is invalid
# 7: the joint acceleration exceeds a limit or is invalid
# 8: the time step does not match robot time step or is invalid
# 9: the trajectory size is outside the limits or is invalid
# 11: the joint position exceeds a limit, initial joint position does not match current robot joint position or is invalid
# 15: the new trajectory cannot start because another trajectory is being played
# 20: the cartesian position exceeds a limit, initial pose does not match current robot pose or is invalid
# 21: the cartesian orientation exceeds a limit, initial orientation does not match current robot orientation or is invalid
# 22: the cartesian linear velocity exceeds a limit or is invalid
# 23: the cartesian angular velocity exceeds a limit or is invalid
# 24: the joint torque exceeds a limit or is invalid
# 25: the waypoints in a trajectory must be all of the same type (cartesian or angular)
# 26: the first waypoint must not have blending (currently unused)
# 27: the last waypoint must not have blending
# 28: the blending radius must be positive and not overlap other blendings or waypoint or is invalid
# 29: the reference frame is invalid or not supported
# 30: the conditions (points, velocities) of the trajectory lead to numerical errors that make the computation impossible
int32 error_name

#List of Protection Zone
string[] protection_zone_list

#List of PresetPose
string[] preset_list

# 0: no command
# 1: servo enable
# 2: brake disable
# 4: clear major fault
# 8: clear minor fault
# 16: protective stop
# 32: force brake release
# 64: ignore
# 128: low gains
# 256: led 0
# 512: led 1
uint16 status
