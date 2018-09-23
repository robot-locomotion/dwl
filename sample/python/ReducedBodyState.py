from __future__ import print_function # python3-style print()
import dwl
import numpy as np


# Construct an instance of the ReducedBodyState class, which wraps the C++ class.
rs = dwl.ReducedBodyState()

# Setting up the reduce state
rs.setCoMSE3(dwl.SE3_RPY(np.array([1., 2., 3.]),
                         np.array([0.5, 0., 0.5])))
rs.setCoMVelocity_W(dwl.Motion(np.array([1., 1., 1.]),
                               np.array([0., 0., 0.])))
rs.setCoMAcceleration_W(dwl.Motion(np.array([1., 1., 1.]),
                               np.array([0., 0., 0.])))
rs.setFootSE3_B('lf_foot', dwl.SE3(np.array([0.5, 0.4, -0.6]),
                                   np.eye(3)))
rs.setFootVelocity_B('lf_foot', dwl.Motion(np.array([0.1, 0., -0.6]),
                                           np.array([0., 0., 0.])))
rs.setFootAcceleration_B('lf_foot', dwl.Motion(np.array([0.1, 0., 0]),
                                           np.array([0., 0., 0.])))



# Getting the CoM states
print('CoM states:')
print('  CoM position:')
print(rs.getCoMSE3())
print('  CoM position (HF):')
print(rs.getCoMSE3_H())

print('  CoM velocity (WF):')
print(rs.getCoMVelocity_W())
print('  CoM velocity (BF):')
print(rs.getCoMVelocity_B())
print('  CoM velocity (HF):')
print(rs.getCoMVelocity_H())




# Getting the feet states
print()
print("Feet states:")
print('  Foot position (BF):')
print(rs.getFootSE3_B('lf_foot'))
print('  Foot position (WF):')
print(rs.getFootSE3_W('lf_foot'))
print('  Foot velocity (BF)')
print(rs.getFootVelocity_B('lf_foot'))
print('  Foot velocity (WF)')
print(rs.getFootVelocity_W('lf_foot'))
print('  Foot velocity (HF)')
print(rs.getFootVelocity_H('lf_foot'))
print('  Foot acceleration (BF)')
print(rs.getFootAcceleration_B('lf_foot'))
print('  Foot acceleration (WF)')
print(rs.getFootAcceleration_W('lf_foot'))
print('  Foot acceleration (HF)')
print(rs.getFootAcceleration_H('lf_foot'))

print()
print('  Feet positions (BF):')
print(rs.getFootSE3_B().asdict())
print('  Feet velocities (BF):')
print(rs.getFootVelocity_B().asdict())
print('  Feet accelerations (BF):')
print(rs.getFootAcceleration_B().asdict())




# Setting up the CoP Position
print()
rs.setCoPPosition_W(np.array([0., 0., -0.6]))
print('CoP position:', rs.getCoPPosition_W().transpose())




# Setting up the support region
print()
support = dwl.SE3Map()
support['lf_foot'] = dwl.SE3(np.array([1., 1., 0.]), np.eye(3))
support['lh_foot'] = dwl.SE3(np.array([1., -1., 0.]), np.eye(3))
support['rf_foot'] = dwl.SE3(np.array([-1., 1., 0.]), np.eye(3))
rs.setSupportRegion(support)



# Printing the reduced state
print()
print(rs)
