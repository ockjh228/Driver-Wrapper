# ProtectionZoneCmd.msg
# 2024.11.06

# Mode of the command:
# UNKNOWN = 0
# CREATE = 1
# DELETE = 2
# CONTROL = 3
uint8 mode

# Unique identifier for the protection zone
int32 zone_id

# Name or label for the protection zone
string zone_name

# Status to activate or deactivate the protection zone
bool is_enabled

# Shape type of the protection zone:
# UNSPECIFIED_SHAPE = 0
# CYLINDER = 1
# SPHERE = 2
# RECTANGULAR_PRISM = 3
int32 shape_type

# Center position of the protection zone:
# [x, y, z]
float32[] position

# Dimensions of the protection zone: 
# [width(x), depth(y), height(z)]
float32[] dimensions

# Orientation (Euler angles in degrees) of the protection zone:
# [roll (x-axis), pitch (y-axis), yaw (z-axis)]
float32[] orientation

# Envelope thickness of the protection zone: 
float32 envelope_thickness #[unit: m]

# Deceleration Target Speed
float32 envelope_target_speed #[unit: m/s]