#!/usr/bin/env python
import rospy
from ctypes import c_ushort
from dwl_msgs.msg import WholeBodyTrajectory
from dwl_msgs.msg import WholeBodyState
from sensor_msgs.msg import JointState


class WholeBodyTrajectoryPublisher():
    def __init__(self):
        # Defining the subscriber
        rospy.Subscriber("whole_body_trajectory", WholeBodyTrajectory, self.callback)
        
        # Defining the publisher
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)


    def callback(self, msg):
        state = WholeBodyState()
        
        # Setting the current state
        state = msg.actual
        
        # Publishing the current joint state
        self.publishJointState(state);
        rospy.sleep(0.5)
        
        # Publishing the joint state trajectory
        for i in range(len(msg.trajectory)):
            duration = msg.trajectory[i].time - state.time
            rospy.sleep(duration)
            state = msg.trajectory[i]
            self.publishJointState(state)
       
        
    def publishJointState(self, state):
        msg = JointState()
        
        # Setting the time
        msg.header.stamp = rospy.Time.now()
        
        # Setting the floating-base system DoF
        num_base_joints = len(state.base)
        num_joints = (num_base_joints + len(state.joints))
        
        # Initializing the joint state sizes
        msg.name = num_joints * [""]
        msg.position = num_joints * [0.0]
        msg.velocity = num_joints * [0.0]
        msg.effort = num_joints * [0.0]
        
        # Setting the whole-body state message
        for i in range(num_joints):
            if i < num_base_joints:
                base_id = state.base[i].id
                msg.name[i] = state.base[i].name
                msg.position[i] = state.base[i].position
                msg.velocity[i] = state.base[i].velocity
                msg.effort[i] = 0.0
            else:
                joint_idx = i - num_base_joints
                msg.name[i] = state.joints[joint_idx].name
                msg.position[i] = state.joints[joint_idx].position;
                msg.velocity[i] = state.joints[joint_idx].velocity;
                msg.effort[i] = state.joints[joint_idx].effort;
                
        # Publishing the current joint state       
        self.pub.publish(msg)
        


if __name__ == '__main__':
    rospy.init_node('whole_body_trajectory_publisher')
        
    jsp = WholeBodyTrajectoryPublisher()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
