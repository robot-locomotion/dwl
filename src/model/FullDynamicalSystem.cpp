#include <model/FullDynamicalSystem.h>


namespace dwl
{

namespace model
{

FullDynamicalSystem::FullDynamicalSystem()
{

}

FullDynamicalSystem::~FullDynamicalSystem()
{

}


void FullDynamicalSystem::compute(Eigen::VectorXd& constraint,
								  const LocomotionState& state)
{
	constraint.resize(2);

	dwl::rbd::BodySelector foot;
	foot.push_back("foot");

	Eigen::VectorXd estimated_joint_forces = state.joint_for;
	dynamics_.computeConstrainedFloatingBaseInverseDynamics(estimated_joint_forces,
															state.base_pos, state.joint_pos,
															state.base_vel, state.joint_vel,
															state.base_acc, state.joint_acc,
															foot);


	constraint = estimated_joint_forces - state.joint_for;
}


void FullDynamicalSystem::computeJacobian(Eigen::MatrixXd& jacobian,
										  const LocomotionState& state)
{

}


void FullDynamicalSystem::getBounds(Eigen::VectorXd& lower_bound,
									Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(2);
	upper_bound = Eigen::VectorXd::Zero(2);
}

} //@namespace model
} //@namespace dwl
