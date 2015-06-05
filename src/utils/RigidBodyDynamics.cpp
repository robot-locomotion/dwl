#include <utils/RigidBodyDynamics.h>


namespace dwl
{

namespace rbd
{

bool isFloatingBaseRobot(const RigidBodyDynamics::Model& model)
{
	bool is_floating_base = false;
	int i = 1;
	while (model.mBodies[i].mIsVirtual) {
		i = model.mu[i][0];
		if (i == 6)
			is_floating_base = true;
	}

	return is_floating_base;
}


Eigen::VectorXd toGeneralizedJointState(const RigidBodyDynamics::Model& model,
										   const Vector6d& base_state,
										   const Eigen::VectorXd& joint_state)
{
	Eigen::VectorXd q(model.dof_count);
	if (isFloatingBaseRobot(model))
		q << base_state, joint_state;
	else
		q = joint_state;

	return q;
}


void fromGeneralizedJointState(const RigidBodyDynamics::Model& model,
								   Vector6d& base_state,
								   Eigen::VectorXd& joint_state,
								   const Eigen::VectorXd& generalized_state)
{
	if (isFloatingBaseRobot(model)) {
		base_state = generalized_state.head<6>();
		joint_state = generalized_state.tail(model.dof_count - 6);
	} else {
		base_state = Vector6d::Zero();
		joint_state = generalized_state;
	}
}


} //@namespace rbd
} //@namespace dwl
