from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np

# Construct an instance of the WholeBodyState class, which wraps the C++ class.
ws = dwl.WholeBodyState()

# Setting up the joint dof
ws.setJointDoF(12)
print("The number of DoFs is ", ws.getJointDof())

# Setting up base states
ws.setBasePosition_W(np.array([1., 2., 3.]))
print("The base position is ", ws.getBasePosition_W().transpose())

ws.setBaseRPY_W(np.array([0.5, 0., 0.]))
print("The base RPY is ", ws.getBaseRPY_W().transpose())
#print("The base orientation is ", ws.getBaseOrientation_W().transpose()) TODO it doesn't work yet


ws.setBaseVelocity_W(np.array([1., 1., 1.]))
print("The base velocity w.r.t. the world frame is ", ws.getBaseVelocity_W().transpose())
print("The base velocity w.r.t. the base frame is ", ws.getBaseVelocity_B().transpose())


ws.setJointPosition(0.5, 11)
print("The joint position is ", ws.getJointPosition().transpose())

ws.setContactPosition_B("lf_foot", np.array([0.1, 0., -0.6]))
print("The lf_foot position w.r.t. the base frame is ", ws.getContactPosition_B("lf_foot").transpose())
print("The lf_foot position w.r.t. the world frame is ", ws.getContactPosition_W("lf_foot").transpose())

ws.setContactVelocity_B("lf_foot", np.array([0.1, 0., -0.6]))
print("The lf_foot velocity w.r.t. the base frame is ", ws.getContactVelocity_B("lf_foot").transpose())
print("The lf_foot velocity w.r.t. the world frame is ", ws.getContactVelocity_W("lf_foot").transpose())
