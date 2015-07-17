#include <model/FullDynamicalSystem.h>


namespace dwl
{

namespace model
{

FullDynamicalSystem::FullDynamicalSystem()
{
	locomotion_variables_.base_pos = true; //TODO remove it
	locomotion_variables_.base_vel = true; //TODO remove it
	locomotion_variables_.base_acc = true; //TODO remove it
	locomotion_variables_.joint_pos = true; //TODO remove it
	locomotion_variables_.joint_vel = true; //TODO remove it
	locomotion_variables_.joint_acc = true; //TODO remove it
	state_dimension_ = 9;
	num_joints_ = 2;
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

	Eigen::VectorXd estimated_joint_forces = Eigen::VectorXd::Zero(2);// = state.joint_for;
	dynamics_.computeConstrainedFloatingBaseInverseDynamics(estimated_joint_forces,
															state.base_pos, state.joint_pos,
															state.base_vel, state.joint_vel,
															state.base_acc, state.joint_acc,
															foot);

//	rbd::Vector6d base_wrench;
//	dynamics_.computeInverseDynamics(base_wrench, estimated_joint_forces,
//									 state.base_pos, state.joint_pos,
//									 state.base_vel, state.joint_vel,
//									 state.base_acc, state.joint_acc);

	std::cout << estimated_joint_forces.transpose() << " = estimated jnt for" << std::endl;
	constraint = estimated_joint_forces;// - state.joint_for;
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
