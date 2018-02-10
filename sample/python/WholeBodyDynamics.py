from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import os
import dwl
import numpy as np

# Configure the printing
np.set_printoptions(suppress=True)

# Construct an instance of the WholeBodyDynamics class, which wraps the C++ class.
ws = dwl.WholeBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()
wdyn = dwl.WholeBodyDynamics()

# Resetting the system from the hyq urdf file
fpath = os.path.dirname(os.path.abspath(__file__))
fbs.resetFromURDFFile(fpath + "/../hyq.urdf", fpath + "/../../config/hyq.yarf")
wdyn.reset(fbs, wkin)

# Define the DoF after initializing the robot model
ws.setJointDoF(fbs.getJointDoF())


# The robot state
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

grf = { 'lh_foot' : np.array([0., 0., 0., 0., 0., 190.778]),
        'rf_foot' : np.array([0., 0., 0., 0., 0., 190.778]),
        'lh_foot' : np.array([0., 0., 0., 0., 0., 190.778]),
        'rh_foot' : np.array([0., 0., 0., 0., 0., 190.778]) };
base_eff = ws.base_eff
joint_eff = ws.joint_eff
base_pos = ws.base_pos
joint_pos = ws.joint_pos
base_vel = ws.base_vel
joint_vel = ws.joint_vel
base_acc = ws.base_acc
joint_acc = ws.joint_acc
noforce = dict()
wdyn.computeInverseDynamics(base_eff, joint_eff,
                            base_pos, joint_pos,
                            base_vel, joint_vel,
                            base_acc, joint_acc,
                            grf);
print("----------------------------- Inverse Dynamics ------------------------------------")
print("Base wrench:", base_eff.transpose())
print("Joint forces:", joint_eff.transpose())


print()
print("------------------------ Floating-based Inverse Dynamics --------------------------")
wdyn.computeFloatingBaseInverseDynamics(base_acc, joint_eff,
                                        base_pos, joint_pos,
                                        base_vel, joint_vel,
                                        joint_acc, grf);

print("Base acceleration:", base_acc.transpose())
print("Joint forces:", joint_eff.transpose())


print()
print("---------------------- Constrained Floating-based Dynamics ------------------------")
wdyn.computeConstrainedFloatingBaseInverseDynamics(joint_eff,
                                                   base_pos, joint_pos,
                                                   base_vel, joint_vel,
                                                   base_acc, joint_acc,
                                                   fbs.getEndEffectorNames(dwl.FOOT));
print("Joint forces:", joint_eff.transpose())


print()
print("--------------------------- Gravitational Wrench ----------------------------------")
com_pos = np.zeros(3)
grav_wrench_W = wdyn.computeGravitoWrench(com_pos)
print("The gravitational wrench:", grav_wrench_W.transpose())


print()
print("---------------------------- Inertial matrices ------------------------------------")
joint_inertial_mat = wdyn.computeJointSpaceInertiaMatrix(base_pos, joint_pos);
print("The joint-space inertial matrix: ", joint_inertial_mat)

com_inertial_mat = wdyn.computeCentroidalInertiaMatrix(base_pos, joint_pos)
print("The centroidal inertial matrix: ", com_inertial_mat)


print()
print("----------------------------- Contact forces --------------------------------------")
contact_forces = dict()
wdyn.computeContactForces(contact_forces, joint_eff,
                          base_pos, joint_pos,
                          base_vel, joint_vel,
                          base_acc, joint_acc,
                          fbs.getEndEffectorNames(dwl.FOOT))
print("The contact forces:", contact_forces)
print("The joint efforts:", joint_eff.transpose())


print()
print("------------------------ Estimated contact forces ---------------------------------")
wdyn.estimateContactForces(contact_forces,
                           base_pos, joint_pos,
                           base_vel, joint_vel,
                           base_acc, joint_acc,
                           joint_eff, fbs.getEndEffectorNames(dwl.FOOT));
print("The contact forces:", contact_forces)


print()
print("-------------------------- Center of pressure -------------------------------------")
cop_pos = np.zeros(3)
contact_forces = { 'lf_foot' : np.array([0., 0., 0., 0., 0., 190.778]),
                   'lh_foot' : np.array([0., 0., 0., 0., 0., 190.778]),
                   'rf_foot' : np.array([0., 0., 0., 0., 0., 190.778]),
                   'rh_foot' : np.array([0., 0., 0., 0., 0., 190.778]) };
contact_pos = { 'lf_foot' : np.array([0.371, 0.207, -0.589]),
                'lh_foot' : np.array([-0.371, 0.207, -0.589]),
                'rf_foot' : np.array([0.371, -0.207, -0.589]),
                'rh_foot' : np.array([-0.371, -0.207, -0.589]) }
wdyn.computeCenterOfPressure(cop_pos,
                             contact_forces,
                             contact_pos);
print("The center of pressure:", cop_pos.transpose())


print()
print("--------------------------- Capture point -----------------------------------------")
icp_pos = np.zeros(3)
com_pos = fbs.getSystemCoM(ws.base_pos, ws.joint_pos)
com_vel = fbs.getSystemCoMRate(ws.base_pos, ws.joint_pos,
                               ws.base_vel, ws.joint_vel)
height = 0.589
wdyn.computeInstantaneousCapturePoint(icp_pos,
                                      com_pos, com_vel,
                                      height);
print("The instantaneous capture point:", icp_pos.transpose())


print()
print("----------------------- Centroidal moment pivot -----------------------------------")
cmp_pos = np.zeros(3)
wdyn.computeCentroidalMomentPivot(cmp_pos,
                                  com_pos, height,
                                  contact_forces);
print("The centroidal moment pivot:", cmp_pos.transpose())


print()
print("----------------------------- CoM torque ------------------------------------------")
com_torque = np.zeros(3)
wdyn.computeCoMTorque(com_torque,
                      cop_pos, cmp_pos,
                      contact_forces);
print("The CoM torque:", com_torque.transpose())


print()
print("----------------------- Estimated GRFs from CoP -----------------------------------")
wdyn.estimateGroundReactionForces(grf,
                                  cop_pos, contact_pos,
                                  fbs.getEndEffectorNames(dwl.FOOT));
print("The estimated GRFs:", grf)


print()
print("------------------------- Estimated active contacts -------------------------------")
force_threshold = 50.
active_contacts = dwl.string_List()
contact_forces = dict()
wdyn.estimateActiveContactsAndForces(active_contacts, contact_forces,
                                     base_pos, joint_pos,
                                     base_vel, joint_vel,
                                     base_acc, joint_acc,
                                     joint_eff, fbs.getEndEffectorNames(dwl.FOOT), # it uses all the end-effector of the system
                                     force_threshold);
print("The active contacts:", active_contacts)
print("The active contact forces:", contact_forces)



print()
print("------------------------------ Active contacts ------------------------------------")
wdyn.getActiveContacts(active_contacts,
                       contact_forces, force_threshold);
print("The active contacts:", active_contacts)
