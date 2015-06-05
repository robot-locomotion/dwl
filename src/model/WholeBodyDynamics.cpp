#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

WholeBodyDynamics::WholeBodyDynamics()
{

}


WholeBodyDynamics::~WholeBodyDynamics()
{

}


void WholeBodyDynamics::modelFromURDF(std::string model_file, bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);

	if (info) {
		std::cout << "Degree of freedom overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot_model_);
		std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(robot_model_);

		std::cout << "Model Hierarchy:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(robot_model_);
	}
}


void WholeBodyDynamics::computeWholeBodyInverseDynamics(Vector6d& base_wrench,
															   Eigen::VectorXd& joint_forces,
															   const Vector6d& g,
															   const Vector6d& base_pos,
															   const Eigen::VectorXd& joint_pos,
															   const Vector6d& base_vel,
															   const Eigen::VectorXd& joint_vel,
															   const Vector6d& base_acc,
															   const Eigen::VectorXd& joint_acc)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc);
	Eigen::VectorXd tau = rbd::toGeneralizedJointState(robot_model_, base_wrench, joint_forces);

	std::cout << q.transpose() << std::endl;
	std::cout << q_dot.transpose() << std::endl;
	std::cout << q_ddot.transpose() << std::endl;

	RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau);

	std::cout << tau.transpose() << std::endl;
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau);
}

} //@namespace model
} //@namespace dwl
