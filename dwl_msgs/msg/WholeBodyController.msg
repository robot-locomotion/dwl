# This represents a locomotion trajectory with reference coordinate frame and timestamp
Header header

# The current whole-body state
dwl_msgs/WholeBodyState desired

# The current whole-body state
dwl_msgs/WholeBodyState actual

# The current whole-body state
dwl_msgs/WholeBodyState error

# Controller states. The command represent the total command send it to the actuator given a feedfoward command
# (operational-space controller) and feedback command (pid controller)
dwl_msgs/JointCommand[] command
