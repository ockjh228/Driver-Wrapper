# ProtectionZoneInfo.msg 
# 2024.11.12

# Name or label for the protection zone (e.g., "ZONE1", "SAFE_AREA")
string zone_name

# Origin (center position) of the protection zone in 3D space [x, y, z]
float64[] origin

# Dimensions of the protection zone:
# [width (x-axis), depth (y-axis), height (z-axis)]
float64[] dimensions

# Shape Type of Protection Zone
int32 shape_type

# Enabled Status of Protection zone
bool is_enabled

# Envelope Thickness (m)
float64 envelope_thickness

# Envelope Target Speed (m/s)
float64 envelope_target_speed 