# PresetCmd.msg
# 2024.11.12

# Mode of the command:
# UNKNOWN = 0
# CREATE = 1
# DELETE = 2
# CONTROL = 3
uint8 mode

# Preset name or target configuration name (e.g., "HOME", "DEFAULT")
string preset_name

# Target joint angles for the preset configuration
float64[] target_joint_angles
