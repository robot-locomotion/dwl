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

	std::cout << "Bodies:" << std::endl;
	for (unsigned int body_id = 0; body_id < robot_model_.mBodies.size(); body_id++) {
		std::string body_name = robot_model_.GetBodyName(body_id);
		std::cout << body_name << std::endl;
	}
	std::cout << "Fixed bodies:" << std::endl;
	unsigned int i = 0;
	for (unsigned int body_id = robot_model_.fixed_body_discriminator;
			body_id < (robot_model_.fixed_body_discriminator + robot_model_.mFixedBodies.size()); body_id++) {
		std::string body_name = robot_model_.GetBodyName(body_id);
		std::cout << body_name << std::endl;
		std::cout << "Movable parent: " << robot_model_.GetBodyName(robot_model_.mFixedBodies[i].mMovableParent) << std::endl;
		std::cout << "Parent tf: " << robot_model_.mFixedBodies[i].mParentTransform << std::endl;
		i++;
	}

}


void WholeBodyDynamics::computeWholeBodyInverseDynamics(rbd::Vector6d& base_wrench,
															   Eigen::VectorXd& joint_forces,
															   const rbd::Vector6d& base_pos,
															   const Eigen::VectorXd& joint_pos,
															   const rbd::Vector6d& base_vel,
															   const Eigen::VectorXd& joint_vel,
															   const rbd::Vector6d& base_acc,
															   const Eigen::VectorXd& joint_acc,
															   const rbd::Vector6d& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	SpatialVector_t fext(ext_force);
//	fext->push_back(SpatialVector_t(ext_force));

	std::cout << "fext1 = " << ext_force.transpose() << std::endl;
	std::cout << "fext2 = " << SpatialVector_t(ext_force).base() << std::endl;

	RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau);//, &fext);

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau);
}

} //@namespace model
} //@namespace dwl
