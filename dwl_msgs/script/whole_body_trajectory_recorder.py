#!/usr/bin/env python
import rospy
import rosbag
import time
from datetime import datetime
from dwl_msgs.msg import WholeBodyTrajectory
from std_msgs.msg import Bool


class WholeBodyTrajectoryRecorder():
    def __init__(self):
        # Defining the subscriber
        rospy.Subscriber("/hyl/constrained_operational_controller/plan", WholeBodyTrajectory, self.callback)


    def callback(self, msg):
        # Getting the current date time
        i = datetime.now()
        
        # Defining the recorder
        bag = rosbag.Bag(i.strftime('%Y-%m-%d-%H-%M-%S') + ".bag", 'w')
        
        # Getting the trajectory duration
        length = len(msg.trajectory) - 1
        duration = msg.trajectory[length].time
        
        try:
            activate = Bool()
            activate.data = True
            bag.write('playing', activate)
        
            time.sleep(duration)
            bag.write('/hyl/constrained_operational_controller/plan', msg)
        finally:
            bag.close()



if __name__ == '__main__':
    rospy.init_node('whole_body_trajectory_recorder')
        
    jsp = WholeBodyTrajectoryRecorder()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
