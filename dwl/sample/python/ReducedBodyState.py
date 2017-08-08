from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np

# Construct an instance of the ReducedBodyState class, which wraps the C++ class.
rs = dwl.ReducedBodyState()


# Setting up the CoM states
rs.setCoMPosition_W(np.array([1., 2., 3.]))
rs.setRPY_W(np.array([0.5, 0., 0.]))
rs.setCoMVelocity_W(np.array([1., 1., 1.]))
print("CoM states:")
print("    com_pos: ", rs.getCoMPosition_W().transpose())
print("    base_RPY: ", rs.getRPY_W().transpose())
#print("The base orientation is ", rs.getOrientation_W().transpose()) TODO it doesn't work yet
print("    com_vel_W: ", rs.getCoMVelocity_W().transpose())
print("    com_vel_B: ", rs.getCoMVelocity_B().transpose())
print("    com_vel_H: ", rs.getCoMVelocity_H().transpose())


# Setting up the feet states
rs.setFootPosition_B("lf_foot", np.array([0.5, 0.4, -0.6]))
rs.setFootPosition_B("lh_foot", np.array([0.5, -0.4, -0.6]))
rs.setFootVelocity_B("lf_foot", np.array([0.1, 0., -0.6]))
rs.setFootVelocity_B("lh_foot", np.array([0.5, -0.4, -0.6]))
print("Feet states:")
print("    lf_foot_pos_B: ", rs.getFootPosition_B("lf_foot").transpose())
print("    lf_foot_pos_W: ", rs.getFootPosition_W("lf_foot").transpose())
print("    lf_foot_vel_B: ", rs.getFootVelocity_B("lf_foot").transpose())
print("    lf_foot_vel_W: ", rs.getFootVelocity_W("lf_foot").transpose())
print("    contact_pos_B: ", rs.getFootPosition_B())
print("    contact_pos_W: ", rs.getFootPosition_W())
print("    contact_pos_H: ", rs.getFootPosition_H())
print("    contact_vel_B: ", rs.getFootVelocity_B())
print("    contact_vel_W: ", rs.getFootVelocity_W())
print("    contact_vel_H: ", rs.getFootVelocity_H())


# Setting up the CoP Position
rs.setCoPPosition_W(np.array([0., 0., -0.6]))
print("CoP positions:")
print("    cop_pos_W: ", rs.getCoPPosition_W().transpose())
