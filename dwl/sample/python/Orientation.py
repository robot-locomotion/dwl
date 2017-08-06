from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np

# Computing the RPY from rotation matrix
rot_mat = np.array([[0., 0., 1.],
                    [0., 1., 0.],
                    [-1., 0., 0.,]])
rpy = dwl.getRPY(rot_mat)
print("The rpy vector:", rpy.transpose())
print("    Roll:", dwl.getRoll(rpy))
print("    Pitch:", dwl.getPitch(rpy))
print("    Yaw:", dwl.getYaw(rpy))

#print("The quaternion from RPY:", dwl.getQuaternion_RPY(rpy))
#print("The quaternion from RotMtx:", dwl.getQuaternion_RM(rot_mat))

# Computing the rotation matrix from RPY
print("The rotation matrix:", dwl.getRotationMatrix(rpy))


# Computing the inverse of Euler rate matrix
print("The inverse euler rate matrix from RPY:", dwl.getInverseEulerAnglesRatesMatrix_RPY(rpy))
print("The inverse euler rate matrix from RotMtx:", dwl.getInverseEulerAnglesRatesMatrix_RM(rot_mat))

# Computing the Euler rate matrix
print("The euler rate matrix from RPY:", dwl.getEulerAnglesRatesMatrix_RPY(rpy))
print("The euler rate matrix from RotMtx:", dwl.getEulerAnglesRatesMatrix_RM(rot_mat))


# Computing the derivatives of the inverse of Euler rate matrix
rpy_d = np.array([0., 0., 0.])
print("The derivatives of the inverse euler rate matrix:", dwl.getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_d))

# Computing the direction cosine matrix
print("The direction cosine matrix: ", dwl.getDirectionCosineMatrix(rpy))