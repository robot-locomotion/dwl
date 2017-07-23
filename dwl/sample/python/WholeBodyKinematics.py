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
jacobian = []#np.array([])#, fixed_jac, floating_jac;
print(jacobian)
print(fbs.getEndEffectorNames(dwl.FOOT))
base_pos = ws.base_pos
joint_pos = ws.joint_pos
wkin.computeJacobian(jacobian,
                     base_pos, joint_pos,
                     fbs.getEndEffectorNames(dwl.FOOT),
                     dwl.Linear)
#    std::cout << "---------------------------------------" << std::endl;
print(jacobian)
#    std::cout << jacobian << " = jacobian" << std::endl;
#    wkin.getFixedBaseJacobian(fixed_jac, jacobian);
#    std::cout << "---------------------------------------" << std::endl;
#    std::cout << fixed_jac << " = fixed jacobian" << std::endl;
#    wkin.getFloatingBaseJacobian(floating_jac, jacobian);
#    std::cout << "---------------------------------------" << std::endl;
#    std::cout << floating_jac << " = floating jacobian" << std::endl << std::endl;