from __future__ import print_function
# This lets us use the python3-style print() function even in python2. It should have no effect if you're already running python3.

import dwl
import numpy as np

# Construct an instance of the FloatingBaseSystem class, which wraps the C++ class.
fbs = dwl.FloatingBaseSystem()
ws = dwl.WholeBodyState()

# Initializing the URDF model and whole-body state
fbs.resetFromURDFFile("../hyq.urdf", "../../config/hyq.yarf")
ws.setJointDoF(fbs.getJointDoF())

# Getting the total mass of the system. Note that you could also get the mass of a specific
# body (e.g. sys.getBodyMass(body_name))
print("Total mass: ", fbs.getTotalMass())
print("lf_upperleg mass: ", fbs.getBodyMass("lf_upperleg"))
print("The gravity acceleration is ", fbs.getGravityAcceleration())


# Getting the CoM of the floating-base body. Note that you could also get the CoM of a
# specific body (e.g. sys.getBodyCoM(body_name))
print("Floating-base CoM: ", fbs.getFloatingBaseCoM().transpose())
print("lf_upperleg CoM: ", fbs.getBodyCoM("lf_upperleg").transpose())


# Getting the number of system DoF, floating-base Dof, joints and end-effectors
print("Total DoF: ", fbs.getSystemDoF())
print("Floating-base DoF: ", fbs.getFloatingBaseDoF());
print("Number of joint: ", fbs.getJointDoF())
print("Number of end-effectors: ", fbs.getNumberOfEndEffectors())
print("The floating-base body name: ", fbs.getFloatingBaseName())


print("lf_kfe_joint ID: ", fbs.getJointId("lf_kfe_joint"))

print("The CoM position: ", fbs.getSystemCoM(ws.base_pos, ws.joint_pos).transpose())
print("The CoM velocity: ", fbs.getSystemCoMRate(ws.base_pos, ws.joint_pos,
												 ws.base_vel, ws.joint_vel).transpose())




# Getting the floating-base information
for i in range(0,6):
	base_joint = fbs.getFloatingBaseJoint(i)
	if (base_joint.active):
		print("Base joint[", base_joint.id, "] = ", base_joint.name)

for name in fbs.getFloatingJointNames():
	print("The base joint names are ", name)

# Getting the joint names and ids
joints = fbs.getJoints()
for name in joints:
	print("Joint[", joints[name], "] = ", name)

for name in fbs.getJointNames():
	print("The joint names are ", name)

# Getting the end-effector names and ids
contacts = fbs.getEndEffectors()
print("The number of feet are ", fbs.getNumberOfEndEffectors(dwl.FOOT))
for name in contacts:
	print("End-effector[", contacts[name], "] = ", name)

# Getting the joint limits
joint_lim = fbs.getJointLimits()
for key in joint_lim:
	print(joint_lim[key])

#print(joint_lim)
#print(joint_lim['lf_haa_joint'])


# Getting the default posture
joint_pos0 = fbs.getDefaultPosture()
print("The nominal joint positions = ", joint_pos0.transpose())

# Converting between whole-body state to generalized state, and viceverse
base_state = np.zeros(6)
joint_state = joint_pos0
generalized_state = fbs.toGeneralizedJointState(base_state, joint_state)
print("Converting the whole-body state to generalized state")
print("Base state = ", base_state.transpose())
print("Joint state = ", joint_state.transpose())
print("The generalized state = ", generalized_state.transpose())
new_base_state = np.zeros(6)
new_joint_state = np.zeros(fbs.getJointDoF())
fbs.fromGeneralizedJointState(new_base_state, new_joint_state, generalized_state);
print("New base state = ", new_base_state.transpose())
print("New joint state = ", new_joint_state.transpose())


# Setting up the branch states
joint_pos = ws.getJointPosition()
lf_branch_pos = np.array([0.5, 0.75, 1.5]);
fbs.setBranchState(joint_pos, lf_branch_pos, "lf_foot");
ws.setJointPosition(joint_pos)
print("Setting up the lf_foot branch position = ", lf_branch_pos.transpose())
print("Joint_position = ", ws.getJointPosition().transpose())
print("The lf_foot branch position = ", fbs.getBranchState(ws.getJointPosition(), "lf_foot").transpose())