#include <model/ConstrainedDynamicalSystem.h>


namespace dwl
{

namespace model
{

ConstrainedDynamicalSystem::ConstrainedDynamicalSystem()
{
	locomotion_variables_.position = true; //TODO remove it
	locomotion_variables_.velocity = true; //TODO remove it
	locomotion_variables_.acceleration = true; //TODO remove it
	state_dimension_ = 9;
	num_joints_ = 2;
}

ConstrainedDynamicalSystem::~ConstrainedDynamicalSystem()
{

}


void ConstrainedDynamicalSystem::compute(Eigen::VectorXd& constraint,
								  const LocomotionState& state)
{
	constraint.resize(2);

	dwl::rbd::BodySelector foot;
	foot.push_back("foot");

	Eigen::VectorXd estimated_joint_forces;
	dynamics_.computeConstrainedFloatingBaseInverseDynamics(estimated_joint_forces,
															state.base_pos, state.joint_pos,
															state.base_vel, state.joint_vel,
															state.base_acc, state.joint_acc,
															foot);


	std::cout << estimated_joint_forces.transpose() << " = estimated jnt for" << std::endl;
	constraint = estimated_joint_forces;// - state.joint_for;
}


void ConstrainedDynamicalSystem::getBounds(Eigen::VectorXd& lower_bound,
									Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(2);
	upper_bound = Eigen::VectorXd::Zero(2);
}

} //@namespace model
} //@namespace dwl
