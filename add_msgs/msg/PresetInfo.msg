# PresetInfo.msg
# 2024.11.12

# Name or label of the preset (e.g., "DEFAULT", "DEFAULT")
string preset_name

# Joint angles for each joint in the preset configuration
float64[] joint_angles

# Pose of the preset in the form [x, y, z, roll, pitch, yaw]
# Position (x, y, z) and orientation (roll, pitch, yaw) in meters and degrees/radians
float64[] pose