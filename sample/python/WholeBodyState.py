from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np

# Construct an instance of the WholeBodyState class, which wraps the C++ class.
ws = dwl.WholeBodyState()

# Setting up the joint dof
ws.setJointDoF(12)
print("The number of DoFs is ", ws.getJointDoF())
 
 
# Setting up the base states
base_se3 = dwl.SE3_RPY(np.array([1., 2., 3.]),
                       np.array([0.1, 0.2, 0.5]))
base_vel = dwl.Motion(np.array([1., 0., 0.]),
                      np.array([1., 0., 0.]))
base_acc = dwl.Motion(np.array([1., 0., 0.]),
                      np.array([1., 0., 0.]))
ws.setBaseSE3(base_se3)
ws.setBaseVelocity_W(base_vel)
ws.setBaseAcceleration_W(base_acc)

print('Base SE3:')
print(' World frame:')
print(' ', ws.getBaseSE3())
print('  base_t: ', ws.getBaseSE3().getTranslation().transpose())
print('  base_R: ', ws.getBaseSE3().getRotation())
print('  base_q: ', ws.getBaseSE3().getQuaternion().transpose())
print('  base_RPY: ', ws.getBaseSE3().getRPY().transpose())
 
print()
print(' Horizontal frame:')
print(' ', ws.getBaseSE3_H())
print('  base_RPY: ', ws.getBaseSE3_H().getRPY().transpose())
 
print()
print('Base velocity in W:')
print(' ', ws.getBaseVelocity_W())
print('Base velocity in B:')
print(' ', ws.getBaseVelocity_B())
print('Base velocity in H:')
print(' ', ws.getBaseVelocity_H())
print(' Base RPY velocity:')
print(' ', ws.getBaseRPYVelocity_W().transpose())

print()
print('Base acceleration in W:')
print(' ', ws.getBaseAcceleration_W())
print('Base acceleration in B:')
print(' ', ws.getBaseAcceleration_B())
print('Base acceleration in H:')
print(' ', ws.getBaseAcceleration_H())
print('Base RPY acceleration:')
print(' ', ws.getBaseRPYAcceleration_W().transpose())

 
# Setting up the joint states
ws.setJointPosition(0.5, 11)
ws.setJointVelocity(1., 11)
print()
print("Joint states:")
print("	joint_pos: ", ws.getJointPosition().transpose())
print("	joint_vel: ", ws.getJointPosition().transpose())
 
 
# Setting up the contact states
ws.setBaseSE3(dwl.SE3_RPY(np.zeros(3), np.array([0., 0., 0.])))
ws.setBaseVelocity_W(dwl.Motion(np.array([0., 0., 0.]),
                                np.array([0., 0., 0.])))
ws.setBaseAcceleration_W(dwl.Motion(np.array([0., 0., 0.]),
                                   np.array([0., 0., 0.])))
ws.setContactSE3_B('lf_foot',
                   dwl.SE3(np.array([0.371, 0.207, -0.59]),
                           np.matrix([[0.796, 0., -0.605],
                                      [0., 1., 0.],
                                      [0.605, 0., 0.796]])))
ws.setContactVelocity_B('lf_foot',
                        dwl.Motion(np.array([-0.635, -0.118, -0.234]),
                                   np.array([-0.2, 1.75, 0.])))
ws.setContactAcceleration_B('lf_foot',
                            dwl.Motion(np.array([-0.588, -0.094, 0.943]),
                                       np.array([0., 0., -0.35])))

print()
print('Contact states:')
print(' Base frame:')
print(' SE3:')
print('  ', ws.getContactSE3_B().asdict())# ws.getContactSE3_B("lf_foot"))
print(' Velocity:')
print('  ', ws.getContactVelocity_B().asdict())
print(' Acceleration:')
print('  ', ws.getContactAcceleration_B().asdict())
print(' World frame:')
print(' SE3:')
print('  ', ws.getContactSE3_W().asdict())
print(' Velocity:')
print('  ', ws.getContactVelocity_W().asdict())
print(' Acceleration:')
print('  ', ws.getContactAcceleration_W().asdict())
print(' Horizontal frame:')
print(' SE3:')
print('  ', ws.getContactSE3_H().asdict())
print(' Velocity:')
print('  ', ws.getContactVelocity_H().asdict())
print(' Acceleration:')
print('  ', ws.getContactAcceleration_H().asdict())

print()
print(ws)
