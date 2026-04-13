"""
Shared physical constants for the F1Tenth platform.

All labs should import from here instead of hardcoding these values.
"""

# Vehicle geometry
WHEELBASE = 0.3302           # rear-axle to front-axle (m)

# LiDAR mount offset from base_link (m)
LIDAR_X = 0.27
LIDAR_Y = 0.0
LIDAR_Z = 0.15

# Steering limits
MAX_STEERING_ANGLE = 0.4189  # rad (~24 deg)

# Simulation relay limits
MAX_SPEED = 3.0              # m/s → throttle 1.0 in sim

# Default topic names (match driver publishers; override with remap if needed)
SCAN_TOPIC  = '/scan'
DRIVE_TOPIC = '/drive'
ODOM_TOPIC  = '/odom'
ACKERMANN_DRIVE_TOPIC = '/ackermann_drive'
