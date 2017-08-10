from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.


import subprocess

# Running the C++ benchmark
print("C++ benchmark:")
args = ("../bin/wif_benchmark")
popen = subprocess.call(args, stdin=None, stdout=None, stderr=None, shell=True)



print("Python benchmark:")
# Running the python benchmark
import dwl
import numpy as np
import time

# Number of iterations
N = 100000

# Construct an instance of the WholeBodyDynamics class, which wraps the C++ class.
ws = dwl.WholeBodyState()
fbs = dwl.FloatingBaseSystem()
wkin = dwl.WholeBodyKinematics()
wdyn = dwl.WholeBodyDynamics()

# Resetting the system from the hyq urdf file
wdyn.modelFromURDFFile("../sample/hyq.urdf", "../config/hyq.yarf")
fbs = wdyn.getFloatingBaseSystem()
wkin = wdyn.getWholeBodyKinematics()


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



startcputime = time.clock()
for x in range(0, N):
    contact_pos_B = wkin.computePosition(base_pos, joint_pos,
                                         fbs.getEndEffectorNames(dwl.FOOT),
                                         dwl.Linear)
cpu_duration = (time.clock() - startcputime) * 1000000;
print("  Forward kinematics: ", cpu_duration / N, "(microsecs, CPU time)")



wkin.setIKSolver(1.0e-12, 0.01, 50)
base_pos_init = np.zeros(6);
joint_pos_init = fbs.getDefaultPosture();
startcputime = time.clock()
for x in range(0, N):
    wkin.computeJointPosition(joint_pos, contact_pos_B, joint_pos_init)
cpu_duration = (time.clock() - startcputime) * 1000000;
print("  Inverse kinematics: ", cpu_duration / N, "(microsecs, CPU time)")



startcputime = time.clock()
for x in range(0, N):
    wdyn.computeInverseDynamics(base_eff, joint_eff,
                                base_pos, joint_pos,
                                base_vel, joint_vel,
                                base_acc, joint_acc,
                                grf);
cpu_duration = (time.clock() - startcputime) * 1000000;
print("  Inverse dynamics: ", cpu_duration / N, "(microsecs, CPU time)")



startcputime = time.clock()
joint_inertial_mat = np.zeros([6 + fbs.getJointDoF(), 6 + fbs.getJointDoF()])
for x in range(0, N):
    wdyn.computeJointSpaceInertialMatrix(joint_inertial_mat,
                                         base_pos, joint_pos);
cpu_duration = (time.clock() - startcputime) * 1000000;
print("  Joint space inertia matrix: ", cpu_duration / N, "(microsecs, CPU time)")