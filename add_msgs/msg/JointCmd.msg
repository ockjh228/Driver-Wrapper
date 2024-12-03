# RemoteJointCmd.msg
# 2024.10.04

# Names of Joint (ex: joint_1, joint_2....joint_6)
string[] name

# Positions of each joint (deg)
float64[] position

# Velocities of each joint (rad/s)
float64[] velocity

# Effort applied to each joint (N*m)
float64[] effort