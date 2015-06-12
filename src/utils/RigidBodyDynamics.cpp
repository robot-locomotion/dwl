#include <utils/RigidBodyDynamics.h>


namespace dwl
{

namespace rbd
{

Part3d angularPart(Vector6d& vector)
{
	return vector.topRows<3>();
}


Part3d linearPart(Vector6d& vector)
{
	return vector.bottomRows<3>();
}


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
	// Note that RBDL defines the floating base state as [linear states, angular states]
	Eigen::VectorXd q(model.dof_count);
	if (isFloatingBaseRobot(model))
		q << base_state.segment(rbd::LX,3), base_state.segment(rbd::AX,3), joint_state;
	else
		q = joint_state;

	return q;
}


void fromGeneralizedJointState(const RigidBodyDynamics::Model& model,
								   Vector6d& base_state,
								   Eigen::VectorXd& joint_state,
								   const Eigen::VectorXd& generalized_state)
{
	// Note that RBDL defines the floating base state as [linear states, angular states]. So, here we convert
	// to the standard convention [angular states, linear states]
	if (isFloatingBaseRobot(model)) {
		base_state << generalized_state.segment(rbd::LX,3), generalized_state.segment(rbd::AX,3);
		joint_state = generalized_state.tail(model.dof_count - 6);
	} else {
		base_state = Vector6d::Zero();
		joint_state = generalized_state;
	}
}


Vector6d convertVelocityToSpatialVelocity(Vector6d& velocity,
											  const Eigen::Vector3d& point)
{
	rbd::Vector6d spatial_velocity;
	spatial_velocity.segment(rbd::AX,3) = angularPart(velocity);
	spatial_velocity.segment(rbd::LX,3) = linearPart(velocity) +
	math::skewSymmentricMatrixFrom3DVector(point) * angularPart(velocity);

	return spatial_velocity;
}


Vector6d convertForceToSpatialForce(Vector6d& force,
									   const Eigen::Vector3d& point)
{
	rbd::Vector6d spatial_force;
	spatial_force.segment(rbd::AX,3) = angularPart(force) +
			math::skewSymmentricMatrixFrom3DVector(point) * linearPart(force);
	spatial_force.segment(rbd::LX,3) = linearPart(force);

	return spatial_force;
}

} //@namespace rbd
} //@namespace dwl
