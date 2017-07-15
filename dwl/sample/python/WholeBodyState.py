from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np

# Construct an instance of the WholeBodyState class, which wraps the C++ class.
ws = dwl.WholeBodyState()
ws.setJointDoF(12)
#base_pos = np.array([1., 2., 3.])
#ws.setBasePosition_W(base_pos)
print(ws.getJointDof())


ws.setJointPosition(0.5, 11)
print(ws.getBasePosition_W())
