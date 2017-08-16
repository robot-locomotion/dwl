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
base_vel = ws.base_vel
joint_vel = ws.joint_vel
base_acc = ws.base_acc
joint_acc = ws.joint_acc
wkin.computeJacobian(jacobian,
                     base_pos, joint_pos,
                     fbs.getEndEffectorNames(dwl.FOOT),
                     dwl.Linear)
print(jacobian, " = Full Jacobian")

wkin.getFixedBaseJacobian(fixed_jac, jacobian);
print(fixed_jac, " = Fixed Jacobian")

wkin.getFloatingBaseJacobian(floating_jac, jacobian);
print(floating_jac, " =  Floating-based Jacobian")


# Computing the contact positions
contact_pos_W = wkin.computePosition(base_pos, joint_pos,
                                     fbs.getEndEffectorNames(dwl.FOOT),
                                     dwl.Linear)
print("The contact position:", contact_pos_W)


# Computing the contact velocities
contact_vel_W = wkin.computeVelocity(base_pos, joint_pos,
                                     base_vel, joint_vel,
                                     fbs.getEndEffectorNames(dwl.FOOT),
                                     dwl.Linear)
print("The contact velocity:", contact_vel_W)


# Computing the contact accelerations
contact_acc_W = wkin.computeAcceleration(base_pos, joint_pos,
                                         base_vel, joint_vel,
                                         base_acc, joint_acc,
                                         fbs.getEndEffectorNames(dwl.FOOT),
                                         dwl.Linear)
print("The contact accelerations:", contact_acc_W)


# Computing the Jdot *qdot of the contacts
contact_jdqd_W = wkin.computeJdotQdot(base_pos, joint_pos,
                                      base_vel, joint_vel,
                                      fbs.getEndEffectorNames(dwl.FOOT),
                                      dwl.Linear)
print("The Jdot*qdot:", contact_jdqd_W)


# Computing the joint positions
wkin.setIKSolver(1.0e-12, 0.01, 50)
joint_pos_init = fbs.getDefaultPosture();
contact_pos_B = { 'lh_foot' : np.array([-0.371,0.207,-0.689]),
                  'rh_foot' : np.array([-0.371,-0.207,-0.589]) }
wkin.computeJointPosition(joint_pos, contact_pos_W, joint_pos_init)
print("The joint positions:", joint_pos.transpose())


# Computing the whole-body inverse kinematics
base_pos_init = np.zeros(6);
wkin.computeInverseKinematics(base_pos, joint_pos, contact_pos_B, base_pos_init, joint_pos_init)
print("The base position:", base_pos.transpose())
print("The joint positions:", joint_pos.transpose())