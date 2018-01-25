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
ws.setBasePosition(np.array([1., 2., 3.]))
ws.setBaseRPY(np.array([0.5, 0., 0.]))
ws.setBaseVelocity_W(np.array([1., 1., 1.]))
print("Base states:")
print("	base_pos: ", ws.getBasePosition().transpose())
print("	base_RPY: ", ws.getBaseRPY().transpose())
#print("The base orientation is ", ws.getBaseOrientation_W().transpose()) TODO it doesn't work yet
print("	base_vel_W: ", ws.getBaseVelocity_W().transpose())
print("	base_vel_B: ", ws.getBaseVelocity_B().transpose())
print("	base_vel_H: ", ws.getBaseVelocity_H().transpose())


# Setting up the joint states
ws.setJointPosition(0.5, 11)
ws.setJointVelocity(1., 11)
print("Joint states:")
print("	joint_pos: ", ws.getJointPosition().transpose())
print("	joint_vel: ", ws.getJointPosition().transpose())


# Setting up the contact states
ws.setContactPosition_B("lf_foot", np.array([0.5, 0.4, -0.6]))
ws.setContactPosition_B("lh_foot", np.array([0.5, -0.4, -0.6]))
ws.setContactVelocity_B("lf_foot", np.array([0.1, 0., -0.6]))
ws.setContactVelocity_B("lh_foot", np.array([0.5, -0.4, -0.6]))
ws.setContactWrench_B("lf_foot", np.array([0., 0., 0., 100., 50., 900.]))
print("Contact states:")
print("	lf_foot_pos_B: ", ws.getContactPosition_B("lf_foot").transpose())
print("	lf_foot_pos_W: ", ws.getContactPosition_W("lf_foot").transpose())
print("	lf_foot_vel_B: ", ws.getContactVelocity_B("lf_foot").transpose())
print("	lf_foot_vel_W: ", ws.getContactVelocity_W("lf_foot").transpose())
print("	contact_pos_B: ", ws.getContactPosition_B())
print("	contact_pos_W: ", ws.getContactPosition_W())
print("	contact_pos_H: ", ws.getContactPosition_H())
print("	contact_vel_B: ", ws.getContactVelocity_B())
print("	contact_vel_W: ", ws.getContactVelocity_W())
print("	contact_vel_H: ", ws.getContactVelocity_H())
print("	lf_foot_wrc_B: ", ws.getContactWrench_B("lf_foot").transpose())
print("	contact_wrc_B: ", ws.getContactWrench_B())
