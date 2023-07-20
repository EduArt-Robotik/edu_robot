uint8 mode
uint8 drive_kinematic
uint8 feature_mode

# defined enums that have to be used for mode
# driving mode
uint8 UNKNOWN=0 # this entry is valid for driving mode and drive kinematic
uint8 INACTIVE=1
uint8 REMOTE_CONTROLLED=2
uint8 FLEET=4
uint8 CHARGING=8

# kinematic mode
uint8 SKID_DRIVE=1
uint8 MECANUM_DRIVE=2

# feature mode
uint8 NONE=0
uint8 COLLISION_AVOIDANCE=1
uint8 COLLISION_AVOIDANCE_OVERRIDE=2
