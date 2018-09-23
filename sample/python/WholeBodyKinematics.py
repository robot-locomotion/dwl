from __future__ import print_function # python3-style print()
import os
import dwl
import numpy as np


# Configure the printing
np.set_printoptions(suppress=True, linewidth=1000)

# Construct an instance of the WholeBodyKinematics class, which wraps the C++ class.
ws = dwl.WholeBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()

# Resetting the system from the hyq urdf file
fpath = str(os.path.dirname(os.path.abspath(__file__)))
fbs.resetFromURDFFile(fpath + "/../../models/hyq.urdf",
                      fpath + "/../../models/hyq.yarf")
wkin.reset(fbs)

# Define the DoF after initializing the robot model
ws.setJointDoF(fbs.getJointDoF())


# The robot state
ws.setBaseSE3(dwl.SE3_RPY(np.zeros(3),
                          np.array([0., 0., 0.])))
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

ws.setJointVelocity(0.2, fbs.getJointId("lf_haa_joint"))
ws.setJointVelocity(0.75, fbs.getJointId("lf_hfe_joint"))
ws.setJointVelocity(1., fbs.getJointId("lf_kfe_joint"))

ws.setJointAcceleration(0.2, fbs.getJointId("lf_haa_joint"))
ws.setJointAcceleration(0.75, fbs.getJointId("lf_hfe_joint"))
ws.setJointAcceleration(1., fbs.getJointId("lf_kfe_joint"))




# Computing the jacobians
print()
print('Stack of Jacobians:')
jacobian = wkin.computeJacobian(ws.getBaseSE3(), ws.getJointPosition(),
                                fbs.getEndEffectorList(dwl.FOOT))
print(' ', jacobian)



 
print()
print('Floating-base Jacobian (LF foot):')
floating_jac = np.zeros([6, 6])
wkin.getFloatingBaseJacobian(floating_jac, jacobian['lf_foot']);
print(' ', floating_jac)
  
print()
print('Fixed Jacobian (LF foot):')
fixed_jac = np.zeros([6, fbs.getJointDoF()]) 
wkin.getFixedBaseJacobian(fixed_jac, jacobian['lf_foot']);
print(' ', fixed_jac)
 
 
 
 
 
# Computing the contact positions
print()
print('Contact SE3:')
contact_pos_W = wkin.computePosition(ws.getBaseSE3(), ws.getJointPosition(),
                                     fbs.getEndEffectorList(dwl.FOOT))
print(' ',contact_pos_W.asdict()['lf_foot'])
ws.setContactSE3_W(contact_pos_W)
 
 
 
 
# Computing the contact velocities
print()
print('Contact velocity:')
contact_vel_W = wkin.computeVelocity(ws.getBaseSE3(), ws.getJointPosition(),
                                     ws.getBaseVelocity_W(), ws.getJointVelocity(),
                                     fbs.getEndEffectorList(dwl.FOOT))
print(' ',contact_vel_W.asdict()['lf_foot'])
ws.setContactVelocity_W(contact_vel_W);
 
 
 
 
# Computing the contact accelerations
print()
print('Contact acceleration:')
contact_acc_W = wkin.computeAcceleration(ws.getBaseSE3(), ws.getJointPosition(),
                                         ws.getBaseVelocity_W(), ws.getJointVelocity(),
                                         ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
                                         fbs.getEndEffectorList(dwl.FOOT))
print(' ', contact_acc_W.asdict()['lf_foot'])
 
 
 
 
# Computing the Jdot *qdot of the contacts
print()
print('Contact Jd*qd:')
contact_jdqd_W = wkin.computeJdQd(ws.getBaseSE3(), ws.getJointPosition(),
                                  ws.getBaseVelocity_W(), ws.getJointVelocity(),
                                  fbs.getEndEffectorList(dwl.FOOT))
print(' ', contact_jdqd_W.asdict())
 
 
 
 
# Computing the joint positions
wkin.setIKSolver(1.0e-12, 50)
joint_pos = np.zeros(fbs.getJointDoF())
contact_pos_B = dwl.SE3Map()
contact_pos_B['lf_foot'] = dwl.SE3(np.array([0.371,0.207,-0.589]), np.eye(3))
contact_pos_B['lh_foot'] = dwl.SE3(np.array([-0.371,0.207,-0.589]), np.eye(3))
joint_pos_init = fbs.getDefaultPosture()
print()
print('Joint position:')
if wkin.computeJointPosition(joint_pos,
                             contact_pos_B,
                             joint_pos_init):
    print(' ', joint_pos.transpose())
else:
    print(' The IK problem could not be solved')
 
 
 
 
# Computing the joint velocities. Note that you can create and dwl.MotioMap()
# object, similar as before
joint_vel = np.zeros(fbs.getJointDoF())
print()
print('Joint velocity:')
contact_vel_B = dwl.MotionMap()
contact_vel_B['lf_foot'] = dwl.Motion(np.zeros((3,1)), np.matrix([ [1.],[0.],[0.] ]))
contact_vel_B['lh_foot'] = dwl.Motion(np.zeros((3,1)), np.matrix([ [0.],[1.],[0.] ]))
contact_vel_B['rf_foot'] = dwl.Motion(np.zeros((3,1)), np.matrix([ [0.],[0.],[1.] ]))
wkin.computeJointVelocity(joint_vel,
                          ws.getJointPosition(),
                          contact_vel_B)
print(' ', joint_vel.transpose())
 
 
 
 
# Computing the joint accelerations
joint_acc = np.zeros(fbs.getJointDoF())
print()
print('Joint acceleration:')
contact_acc_B = contact_acc_W
wkin.computeJointAcceleration(joint_acc,
                              ws.getJointPosition(),
                              ws.getJointVelocity(),
                              contact_acc_B)
print(' ', joint_acc.transpose())
 
 
 
 
# Computing the CoM position, velocity and acceleration
print()
print('CoM position:')
print(' ', wkin.computeCoM(ws.getBaseSE3(), ws.getJointPosition()).transpose())
c_pos = np.zeros(3)
c_vel = np.zeros(3)
c_acc = np.zeros(3)
wkin.computeCoMRate(c_pos, c_vel, c_acc,
                    ws.getBaseSE3(), ws.getJointPosition(),
                    ws.getBaseVelocity_W(), ws.getJointVelocity(),
                    ws.getBaseAcceleration_W(), ws.getJointAcceleration())
print('CoM velocity:')
print(' ', c_vel)
print('CoM acceleration:')
print(' ', c_acc)
 
 
 
 
print()
print('Constrained acceleration')
ws.setBaseAcceleration_W(dwl.Motion(np.matrix([ [1.],[0.],[0.] ]),
                                    np.matrix([ [0.],[0.],[0.] ])))
joint_facc = np.zeros((fbs.getJointDoF(),1))
wkin.computeConstrainedJointAcceleration(joint_facc,
                                         ws.getBaseSE3(), ws.getJointPosition(),
                                         ws.getBaseVelocity_W(), ws.getJointVelocity(),
                                         ws.getBaseAcceleration_W(),
                                         fbs.getEndEffectorList(dwl.FOOT))
print(' joint acceleration:', joint_facc.transpose())