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
	kinematics_.modelFromURDF(model_file, false);

	if (info) {
		std::cout << "Degree of freedom overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot_model_);
		std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(robot_model_);

		std::cout << "Model Hierarchy:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(robot_model_);
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
															   const rbd::EndEffectorForce& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau, &fext);

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau);
}



void WholeBodyDynamics::computeWholeBodyInverseDynamics(rbd::Vector6d& base_acc,
															   Eigen::VectorXd& joint_forces,
															   const rbd::Vector6d& base_pos,
															   const Eigen::VectorXd& joint_pos,
															   const rbd::Vector6d& base_vel,
															   const Eigen::VectorXd& joint_vel,
															   const Eigen::VectorXd& joint_acc,
															   const rbd::EndEffectorForce& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::Math::SpatialVector base_ddot = RigidBodyDynamics::Math::SpatialVector(base_acc);
	rbd::FloatingBaseInverseDynamics(robot_model_, q, q_dot, q_ddot, base_ddot, tau, &fext);

	// Converting the base acceleration
	base_acc = base_ddot;

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::Vector6d base_wrench;
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau);
}


void WholeBodyDynamics::computeConstrainedWholeBodyInverseDynamics(Eigen::VectorXd& joint_forces,
																			const rbd::Vector6d& base_pos,
																			const Eigen::VectorXd& joint_pos,
																			const rbd::Vector6d& base_vel,
																			const Eigen::VectorXd& joint_vel,
																			const rbd::Vector6d& base_acc,
																			const Eigen::VectorXd& joint_acc,
																			const rbd::EndEffectorSelector& contacts) //TODO finish it
{
	// Computing the feasible accelerations
//	Eigen::VectorXd op_acc;
//	kinematics_.computeWholeBodyAcceleration(op_acc, base_pos, joint_pos, base_vel, joint_vel,
//											 base_acc, joint_acc, contacts, dwl::rbd::Full);
//	Eigen::VectorXd jacd_qd;
//	kinematics_.computeWholeBodyJdotQdot(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, contacts, dwl::rbd::Full);
//	std::cout << jacd_qd << " = jacd_qd" << std::endl;
//
//	rbd::Vector6d base_feas_acc = base_acc;
//	Eigen::VectorXd joint_feas_acc = math::pseudoInverse(jacobian) * (op_acc - jacd_qd);
//	std::cout << base_feas_acc << " = base_feas_acc" << std::endl;
//	std::cout << joint_feas_acc << " = joint_feas_acc" << std::endl;


	rbd::Vector6d base_feas_acc = base_acc;
	Eigen::VectorXd joint_feas_acc = joint_acc;



	// Computing the base wrench assuming a fully actuation on the floating-base
	rbd::Vector6d base_wrench;
	computeWholeBodyInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
									base_vel, joint_vel, base_feas_acc, joint_feas_acc);

	// Computing the base contribution of contact jacobian
	Eigen::MatrixXd base_contact_jac, full_jacobian;
	kinematics_.computeWholeBodyJacobian(full_jacobian, base_pos, joint_pos, contacts, dwl::rbd::Linear);
	kinematics_.getFloatingBaseJacobian(base_contact_jac, full_jacobian);

	// Computing the contact forces that generates the desired base wrench
	Eigen::VectorXd linear_contact_forces =
			math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * base_wrench;
	rbd::EndEffectorForce contact_forces;
	for (unsigned int i = 0; i < contacts.size(); i++)
		contact_forces[contacts[i]] << 0., 0., 0., linear_contact_forces.segment(3*i, 3);


	// Computing the inverse dynamic algorithm
	computeWholeBodyInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
									base_vel, joint_vel, base_feas_acc, joint_feas_acc, contact_forces);
	std::cout << "base wrench = " << base_wrench.transpose() << std::endl;
	std::cout << "joint forces = " << joint_forces.transpose() << std::endl;

//	rbd::Vector6d base_acc2;
//	joint_forces.setZero();
//	computeWholeBodyInverseDynamics(base_acc2, joint_forces, base_pos,
//									joint_pos, base_vel, joint_vel,
//									joint_acc, contact_forces);
//	std::cout << "base acc = " << base_acc2.transpose() << std::endl;
//	std::cout << "joint forces = " << joint_forces.transpose() << std::endl;
}


void WholeBodyDynamics::convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& fext,
													 const rbd::EndEffectorForce& ext_force,
													 const Eigen::VectorXd& q)
{
	// Computing the applied external spatial forces for every body
	fext.resize(robot_model_.mBodies.size());
	// Searching over the movable bodies
	for (unsigned int body_id = 0; body_id < robot_model_.mBodies.size(); body_id++) {
		std::string body_name = robot_model_.GetBodyName(body_id);

		if (ext_force.count(body_name) > 0) {
			// Converting the applied force to spatial force vector in base coordinates
			rbd::Vector6d spatial_force = robot_model_.X_base[6].toMatrixAdjoint() * ext_force.at(body_name);

			fext.at(body_id) = spatial_force;
		} else
			fext.at(body_id) = rbd::Vector6d::Zero();
	}

	// Searching over the fixed bodies
	for (unsigned int it = 0; it < robot_model_.mFixedBodies.size(); it++) {
		unsigned int body_id = it + robot_model_.fixed_body_discriminator;
		std::string body_name = robot_model_.GetBodyName(body_id);
		if (ext_force.count(body_name) > 0) {
			// Converting the applied force to spatial force vector in base coordinates
			rbd::Vector6d spatial_force = robot_model_.X_base[6].toMatrixAdjoint() * ext_force.at(body_name);

			unsigned parent_id = robot_model_.mFixedBodies[it].mMovableParent;
			fext.at(parent_id) += spatial_force;
		}
	}
}

} //@namespace model
} //@namespace dwl
