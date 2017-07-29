from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np


# Construct an instance of the WholeBodyKinematics class, which wraps the C++ class.
ws = dwl.WholeBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()

# Resetting the system from the hyq urdf file
wkin.modelFromURDFFile("../hyq.urdf", "../../config/hyq.yarf")
fbs = wkin.getFloatingBaseSystem()

# Define the DoF after initializing the robot model
ws.setJointDoF(fbs.getJointDoF())


# The robot state
ws.setBasePosition_W(np.array([0., 0., 0.]))
ws.setBaseRPY_W(np.array([0., 0., 0.]))
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


# Computing the jacobians
jacobian = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), 6 + fbs.getJointDoF()])
fixed_jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), fbs.getJointDoF()])
floating_jac = np.zeros([3 * fbs.getNumberOfEndEffectors(dwl.FOOT), 6])
base_pos = ws.base_pos
joint_pos = ws.joint_pos
wkin.computeJacobian(jacobian,
                     base_pos, joint_pos,
                     fbs.getEndEffectorNames(dwl.FOOT),
                     dwl.Linear)
print(jacobian, " = Full Jacobian")

wkin.getFixedBaseJacobian(fixed_jac, jacobian);
print(fixed_jac, " = Fixed Jacobian")

wkin.getFloatingBaseJacobian(floating_jac, jacobian);
print(floating_jac, " =  Floating-based Jacobian")

#dwl::rbd::BodyVectorXd contact_pos_W;
contact_pos_W = dict([("lf_foot", np.array([0,0,0])), ("lh_foot", np.array([0,0,0])), ("rf_foot", np.array([0,0,0])), ("rh_foot", np.array([0,0,0]))])
#print(contact_pos_W)
#contact_pos_W = dwl.map_string_vectorxd({ 'lf_foot' : np.array([0,0,0]), 'lh_foot' : np.array([0,0,0]), 'rf_foot' : np.array([0,0,0]), 'rh_foot' : np.array([0,0,0])})
#contact_pos_W = {}
#contact_pos_W["lf_foot"] = np.zeros(3)
#contact_pos_W["lh_foot"] = np.zeros(3)
#contact_pos_W["rf_foot"] = np.zeros(3)
#contact_pos_W["rh_foot"] = np.zeros(3)
wkin.computeForwardKinematics(contact_pos_W,
                              base_pos, joint_pos,
                              fbs.getEndEffectorNames(dwl.FOOT),
                              dwl.Linear) #dwl.RollPitchYaw);