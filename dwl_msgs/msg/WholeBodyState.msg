# This represent the time state
float64 time

# Floating-base identifiers
uint8 AX=0
uint8 AY=1
uint8 AZ=2
uint8 LX=3
uint8 LY=4
uint8 LZ=5

# This represents the base state (position, velocity, acceleration and effort)
uint8[] base_ids
string[] base_names
dwl_msgs/JointState[] base

# This represents the joint state (position, velocity, acceleration and effort)
string[] joint_names
dwl_msgs/JointState[] joints

# TODO adds contacts forces
