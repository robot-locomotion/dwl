from __future__ import print_function # python3-style print()
import os
import dwl
import numpy as np


# Configure the printing
np.set_printoptions(suppress=True, linewidth=1000)

# Construct an instance of the WholeBodyDynamics class, which wraps the C++ class.
ws = dwl.WholeBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()
wdyn = dwl.WholeBodyDynamics()

# Resetting the system from the hyq urdf file
fpath = str(os.path.dirname(os.path.abspath(__file__)))
fbs.resetFromURDFFile(fpath + "/../../models/hyq.urdf",
                      fpath + "/../../models/hyq.yarf")
wdyn.reset(fbs, wkin)
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




print()
print('Inverse dynamics')
base_eff = ws.getBaseWrench_W()
joint_eff = ws.getJointEffort()
grfs = { 'lf_foot': dwl.Force(np.matrix([ [0.],[0.],[190.778] ]), np.zeros((3,1))),
         'lh_foot': dwl.Force(np.matrix([ [0.],[0.],[190.778] ]), np.zeros((3,1))),
         'rf_foot': dwl.Force(np.matrix([ [0.],[0.],[190.778] ]), np.zeros((3,1))),
         'rh_foot': dwl.Force(np.matrix([ [0.],[0.],[190.778] ]), np.zeros((3,1))) };
wdyn.computeInverseDynamics(base_eff, joint_eff,
                            ws.getBaseSE3(), ws.getJointPosition(),
                            ws.getBaseVelocity_W(), ws.getJointVelocity(),
                            ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
                            grfs);
print(' base wrench:')
print('  ', base_eff)
print(' joint forces:')
print('  ', joint_eff.transpose())




print()
print('Constrained inverse dynamics')
joint_forces = np.zeros((fbs.getJointDoF(),1))
joint_acc = np.zeros((fbs.getJointDoF(),1))
contact_forces = dwl.ForceMap()
ws.setBaseAcceleration_W(dwl.Motion(np.matrix([ [1.],[0.],[0.] ]),
                                    np.matrix([ [0.],[0.],[0.] ])))
wdyn.computeConstrainedInverseDynamics(joint_forces, joint_acc, contact_forces,
                                       ws.getBaseSE3(), ws.getJointPosition(),
                                       ws.getBaseVelocity_W(), ws.getJointVelocity(),
                                       ws.getBaseAcceleration_W(),
                                       fbs.getEndEffectorList(dwl.FOOT));
print(' joint forces:', joint_forces.transpose())
print(' joint accelerations:', joint_acc.transpose())
print(' contact forces:', contact_forces.asdict())




print()
print('Contact forces and joint accelerations')
contact_forces = dwl.ForceMap()
joint_acc = ws.getJointAcceleration() # in case we want to ovewrite the active branches
wdyn.computeContactForces(contact_forces, joint_acc,
                          ws.getBaseSE3(), ws.getJointPosition(),
                          ws.getBaseVelocity_W(), ws.getJointVelocity(),
                          ws.getBaseAcceleration_W(), fbs.getEndEffectorList(dwl.FOOT))
print(' contact forces:', contact_forces.asdict())
print(' joint accelerations:', joint_acc.transpose())




print()
print('Joint-space Inertia Matrix:')
M = wdyn.computeJointSpaceInertiaMatrix(ws.getBaseSE3(), ws.getJointPosition())
print('', M)





print()
print('Centroidal Inertia Matrix:')
H = wdyn.computeCentroidalInertiaMatrix(ws.getBaseSE3(), ws.getJointPosition(),
                                        ws.getBaseVelocity_W(), ws.getJointVelocity())
print('', H)




print()
print('Centroidal Momentum Matrix:')
H = wdyn.computeCentroidalMomentumMatrix(ws.getBaseSE3(), ws.getJointPosition(),
                                         ws.getBaseVelocity_W(), ws.getJointVelocity())
print('', H)




print()
print('Gravito Wrench:')
grav_wrench = wdyn.computeGravitoWrench(ws.getBaseSE3(), ws.getJointPosition())
print('', grav_wrench)




print()
print('Estimated contact forces:')
est_forces = dwl.ForceMap()
wdyn.estimateContactForces(est_forces,
                           ws.getBaseSE3(), ws.getJointPosition(),
                           ws.getBaseVelocity_W(), ws.getJointVelocity(),
                           ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
                           ws.getJointEffort(), fbs.getEndEffectorList(dwl.FOOT))
print('', est_forces.asdict())




print()
print('Estimated GRFs from CoP:')
grfs = dwl.ForceMap()
cop_pos = np.matrix([ [0.],[0.],[-0.58] ])
contact_pos = wkin.computePosition(ws.getBaseSE3(), ws.getJointPosition(),
                                     fbs.getEndEffectorList(dwl.FOOT))
wdyn.estimateGroundReactionForces(grfs,
                                  cop_pos, contact_pos,
                                  fbs.getEndEffectorList(dwl.FOOT))
print('', grfs.asdict())




print()
print('Active contacts and forces')
force_threshold = 50.
active_contacts = dwl.stringList()
est_forces = dwl.ForceMap()
ws.setJointEffort(joint_eff)
wdyn.estimateActiveContactsAndForces(active_contacts, est_forces,
                                     ws.getBaseSE3(), ws.getJointPosition(),
                                     ws.getBaseVelocity_W(), ws.getJointVelocity(),
                                     ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
                                     ws.getJointEffort(), fbs.getEndEffectorList(dwl.FOOT), # it uses all the end-effector of the system
                                     force_threshold)
print(' contacts:', active_contacts)
print(' forces:')
print('  ', est_forces.asdict())




print()
print('Active contacts:')
est_forces['lf_foot'] = dwl.Force()
wdyn.getActiveContacts(active_contacts,
                       est_forces, force_threshold);
print('', active_contacts)


 
 
print()
print('Center of Pressure:')
cop_pos = np.zeros((3,1))
f = dwl.Force(np.array([ [0.], [0.], [190.778] ]), np.zeros((3,1)))
contact_forces = { 'lf_foot' : f,
                   'lh_foot' : f,
                   'rf_foot' : f,
                   'rh_foot' : f };
wdyn.computeCenterOfPressure(cop_pos,
                             contact_forces,
                             contact_pos)
print(' ', cop_pos.transpose())




print()
print('Zero Moment Point:')
cop_pos = np.zeros(3)
f = dwl.Force(np.array([ [0.], [0.], [190.778] ]), np.zeros((3,1)))
contact_forces = { 'lf_foot' : f,
                   'lh_foot' : f,
                   'rf_foot' : f,
                   'rh_foot' : f };
zmp_pos = np.zeros((3,1))
c_pos = np.zeros((3,1))
c_vel = np.zeros((3,1))
c_acc = np.zeros((3,1))
height = 0.589
wkin.computeCoMRate(c_pos, c_vel, c_acc,
                    ws.getBaseSE3(), ws.getJointPosition(),
                    ws.getBaseVelocity_W(), ws.getJointVelocity(),
                    ws.getBaseAcceleration_W(), ws.getJointAcceleration())
wdyn.computeZeroMomentPoint(zmp_pos, c_pos, c_acc, height)
print(' ', zmp_pos.transpose())




print()
print('Instantaneous Capture Point:')
icp_pos = np.zeros((3,1))
wdyn.computeInstantaneousCapturePoint(icp_pos,
                                      c_pos, c_vel,
                                      height)
print(' ', icp_pos.transpose())




print()
print('Centroidal Moment Pivot:')
cmp_pos = np.zeros((3,1))
wdyn.computeCentroidalMomentPivot(cmp_pos,
                                  c_pos, height,
                                  contact_forces)
print(' ', cmp_pos.transpose())


 

print()
print('CoM torque:')
com_torque = np.zeros((3,1))
wdyn.computeCoMTorque(com_torque,
                      c_pos, cmp_pos,
                      contact_forces)
print(' ', com_torque.transpose())