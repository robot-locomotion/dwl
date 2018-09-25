from __future__ import print_function # python3-style print()
import os
import dwl
import numpy as np


# Construct an instance of the WholeBodyState and ReducedBodyState classes,
# which wraps the C++ classes.
ws = dwl.WholeBodyState()
rs = dwl.ReducedBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()
wdyn = dwl.WholeBodyDynamics()

# Construct an instance of the RobotStates class
rob = dwl.RobotStates()

# Creating the whole-body dynamics model
fpath = str(os.path.dirname(os.path.abspath(__file__)))
fbs.resetFromURDFFile(fpath + "/../../models/hyq.urdf",
                      fpath + "/../../models/hyq.yarf")
wkin.reset(fbs)
wdyn.reset(fbs, wkin)


# Define the DoF after initializing the robot model
ws.setJointDoF(fbs.getJointDoF())

# Reseting the model of the RobotStates class
rob.reset(fbs, wkin, wdyn)
rob.setForceThreshold(50) # Nm


# The whole-body state
ws.setBaseSE3(dwl.SE3_RPY(np.array([0., 0., 0.]),
                          np.array([0.5, 0.5, 0.])))
ws.setBaseVelocity_W(dwl.Motion(np.array([0., 0., 0.]),
                                np.array([0., 0., 0.])))
ws.setBaseAcceleration_W(dwl.Motion(np.array([0., 0., 0.]),
                                    np.array([0., 0., 0.])))
ws.setJointPosition(0.75, fbs.getJointId("lf_hfe_joint"))
ws.setJointPosition(-1.5, fbs.getJointId("lf_kfe_joint"))
ws.setJointPosition(-0.75, fbs.getJointId("lh_hfe_joint"))
ws.setJointPosition(1.5, fbs.getJointId("lh_kfe_joint"))
ws.setJointPosition(0.75, fbs.getJointId("rf_hfe_joint"))
ws.setJointPosition(-1.5, fbs.getJointId("rf_kfe_joint"))
ws.setJointPosition(-0.75, fbs.getJointId("rh_hfe_joint"))
ws.setJointPosition(1.5, fbs.getJointId("rh_kfe_joint"))
ws.setContactSE3_W(
    wkin.computePosition(ws.getBaseSE3(), ws.getJointPosition(),
                         fbs.getEndEffectorList(dwl.FOOT)))
ws.setContactVelocity_B({ 'lf_foot' : dwl.Motion(),
                          'lh_foot' : dwl.Motion(),
                          'rf_foot' : dwl.Motion(),
                          'rh_foot' : dwl.Motion() })
ws.setContactAcceleration_B({ 'lf_foot' : dwl.Motion(),
                              'lh_foot' : dwl.Motion(),
                              'rf_foot' : dwl.Motion(),
                              'rh_foot' : dwl.Motion() })
force = dwl.Force(np.array([0.,0.,208.21]), np.zeros(3))
ws.setContactWrench_B({ 'lf_foot' : force,
                        'lh_foot' : force,
                        'rf_foot' : force,
                        'rh_foot' : force })
print('Converting whole-body state to reduced-body state:')
print(rob.getReducedBodyState(ws))




# The reduced-body state
rs.setCoMSE3(dwl.SE3_RPY(np.array([0., 0., 0.]),
                         np.array([0.5, 0.5, 0.])))
rs.setCoMVelocity_B(dwl.Motion(np.array([1., 0., 0.]),
                               np.array([0., 0., 0.])))
rs.setCoMAcceleration_B(dwl.Motion(np.array([1., 0., 0.]),
                                   np.array([0., 0., 0.])))
rs.setFootSE3_W(
    wkin.computePosition(rs.getCoMSE3(), ws.getJointPosition(),
                         fbs.getEndEffectorList(dwl.FOOT)))
rs.setFootVelocity_B('lf_foot', dwl.Motion(np.array([1., 0., 0.]),
                                           np.array([0., 0., 0.])))
rs.setFootAcceleration_B('lf_foot', dwl.Motion(np.array([1., 0., 0.]),
                                               np.array([0., 0., 0.])))
print()
print('Converting reduced-body state to whole-body state:')
print(rob.getWholeBodyState(rs))