# This represent the time state
float64 time

# This represents the base state (position, velocity, acceleration and effort)
dwl_msgs/BaseState[] base

# This represents the joint state (position, velocity, acceleration and effort)
dwl_msgs/JointState[] joints

# This represents the end-effector state (cartesian position and contact forces)
dwl_msgs/ContactState[] contacts
