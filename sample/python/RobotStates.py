from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import os
import dwl
import numpy as np


# Construct an instance of the WholeBodyState and ReducedBodyState classes, which wraps the C++ classes.
ws = dwl.WholeBodyState()
rs = dwl.ReducedBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()
wdyn = dwl.WholeBodyDynamics()

# Construct an instance of the RobotStates class
rob = dwl.RobotStates()

# Creating the whole-body dynamics model
fpath = os.path.dirname(os.path.abspath(__file__))
fbs.resetFromURDFFile(fpath + "/../hyq.urdf", fpath + "/../../config/hyq.yarf")
wdyn.reset(fbs, wkin)

# Define the DoF after initializing the robot model
ws.setJointDoF(fbs.getJointDoF())

# Reseting the model of the RobotStates class
rob.reset(fbs, wkin, wdyn)
rob.setForceThreshold(50) # Nm


# The whole-body state
ws.setBasePosition(np.array([0., 0., 0.]))
ws.setBaseRPY(np.array([0., 0., 0.]))
ws.setBaseVelocity_W(np.array([0., 0., 0.]))
ws.setBaseRPYVelocity_W(np.array([0., 0., 0.]))
ws.setBaseAcceleration_W(np.array([0., 0., 0.]))
ws.setBaseRPYAcceleration_W(np.array([0., 0., 0.]))
ws.setJointPosition(0.75, fbs.getJointId("lf_hfe_joint"))
ws.setJointPosition(-1.5, fbs.getJointId("lf_kfe_joint"))
ws.setJointPosition(-0.75, fbs.getJointId("lh_hfe_joint"))
ws.setJointPosition(1.5, fbs.getJointId("lh_kfe_joint"))
ws.setJointPosition(0.75, fbs.getJointId("rf_hfe_joint"))
ws.setJointPosition(-1.5, fbs.getJointId("rf_kfe_joint"))
ws.setJointPosition(-0.75, fbs.getJointId("rh_hfe_joint"))
ws.setJointPosition(1.5, fbs.getJointId("rh_kfe_joint"))
ws.setContactPositionDict_B({ 'lf_foot' : np.array([0.371, 0.207, -0.589]),
                              'lh_foot' : np.array([-0.371, 0.207, -0.589]),
                              'rf_foot' : np.array([0.371, -0.207, -0.589]),
                              'rh_foot' : np.array([-0.371, -0.207, -0.589]) })
ws.setContactVelocityDict_B({ 'lf_foot' : np.array([0., 0., 0.]),
                              'lh_foot' : np.array([0., 0., 0.]),
                              'rf_foot' : np.array([0., 0., 0.]),
                              'rh_foot' : np.array([0., 0., 0.]) })
ws.setContactAccelerationDict_B({ 'lf_foot' : np.array([0., 0., 0.]),
                                  'lh_foot' : np.array([0., 0., 0.]),
                                  'rf_foot' : np.array([0., 0., 0.]),
                                  'rh_foot' : np.array([0., 0., 0.]) })
ws.setContactWrenchDict_B({ 'lf_foot' : np.array([0., 0., 0., 0., 0., 208.21]),
                            'lh_foot' : np.array([0., 0., 0., 0., 0., 208.21]),
                            'rf_foot' : np.array([0., 0., 0., 0., 0., 208.21]),
                            'rh_foot' : np.array([0., 0., 0., 0., 0., 208.21]) })

rs = rob.getReducedBodyState(ws)


print(rs)

print(ws)
