#!/usr/bin/env python
import rospy
import rosbag
from datetime import datetime
from ctypes import c_ushort
from sys import exit
from dwl_msgs.msg import WholeBodyTrajectory


class WholeBodyTrajectoryRecorder():
    def __init__(self):
        # Defining the subscriber
        rospy.Subscriber("/hyl/constrained_operational_controller/plan", WholeBodyTrajectory, self.callback)


    def callback(self, msg):
        i = datetime.now()
        # Defining the recorder
        bag = rosbag.Bag(i.strftime('%Y-%m-%d-%H-%M-%S') + ".bag", 'w')
        
        try:
            bag.write('/hyl/constrained_operational_controller/plan', msg)
        finally:
            bag.close()
            exit(0)



if __name__ == '__main__':
    rospy.init_node('whole_body_trajectory_recorder')
        
    jsp = WholeBodyTrajectoryRecorder()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
