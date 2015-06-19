#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

WholeBodyDynamics::WholeBodyDynamics() : reduced_base_(NULL)
{

}


WholeBodyDynamics::~WholeBodyDynamics()
{

}


void WholeBodyDynamics::modelFromURDF(std::string model_file, struct rbd::ReducedFloatingBase* reduce_base, bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);
	reduced_base_ = reduce_base;

	kinematics_.modelFromURDF(model_file, reduce_base, false);

	if (info) {
		std::cout << "Degree of freedom overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot_model_);

		std::cout << "Body origins overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(robot_model_);

		std::cout << "Model Hierarchy:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(robot_model_);
	}
}


void WholeBodyDynamics::computeInverseDynamics(rbd::Vector6d& base_wrench,
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
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos, reduced_base_);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel, reduced_base_);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc, reduced_base_);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau, &fext);

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau, reduced_base_);
}



void WholeBodyDynamics::computeFloatingBaseInverseDynamics(rbd::Vector6d& base_acc,
														   Eigen::VectorXd& joint_forces,
														   const rbd::Vector6d& base_pos,
														   const Eigen::VectorXd& joint_pos,
														   const rbd::Vector6d& base_vel,
														   const Eigen::VectorXd& joint_vel,
														   const Eigen::VectorXd& joint_acc,
														   const rbd::EndEffectorForce& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos, reduced_base_);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel, reduced_base_);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc, reduced_base_);
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
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau, reduced_base_);
}

void WholeBodyDynamics::computeConstrainedInverseDynamics(Eigen::VectorXd& joint_forces,
		  const rbd::Vector6d& base_pos,
		  const Eigen::VectorXd& joint_pos,
		  const rbd::Vector6d& base_vel,
		  const Eigen::VectorXd& joint_vel,
		  const rbd::Vector6d& base_acc,
		  const Eigen::VectorXd& joint_acc,
		  const rbd::EndEffectorSelector& contacts)
{
	if (reduced_base_ == NULL) {
		printf(RED "Error: this is not a floating-base platform\n" COLOR_RESET);
		return;
	}

	// Computing the feasible accelerations //TODO figure out about this topic
	rbd::Vector6d base_feas_acc = base_acc;
	Eigen::VectorXd joint_feas_acc = joint_acc;


	// Computing the base wrench assuming a fully actuation on the floating-base
	rbd::Vector6d base_wrench;
	computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_feas_acc, joint_feas_acc);




	// Computing the base contribution of contact jacobian
	Eigen::MatrixXd base_contact_jac, full_jacobian;
	kinematics_.computeJacobian(full_jacobian, base_pos, joint_pos, contacts, dwl::rbd::Linear);
	kinematics_.getFloatingBaseJacobian(base_contact_jac, full_jacobian);

	std::cout << full_jacobian << " = full_jac" << std::endl;
	std::cout << base_contact_jac << " = virtual_contact_jac" << std::endl;


	// Computing the contact forces that generates the desired base wrench in the case of n dof floating-base, where
	// n is less than 6. Note that we describe this floating-base as an unactuated virtual floating-base joints
	rbd::EndEffectorForce contact_forces;
	Eigen::VectorXd endeffector_forces =
			math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * rbd::linearFloatingBaseState(base_wrench);
	unsigned int num_active_contacts = contacts.size();
	for (unsigned int i = 0; i < num_active_contacts; i++)
		contact_forces[contacts[i]] << 0., 0., 0., endeffector_forces.segment(3*i, 3);



}


void WholeBodyDynamics::computeConstrainedFloatingBaseInverseDynamics(Eigen::VectorXd& joint_forces,
																	  const rbd::Vector6d& base_pos,
																	  const Eigen::VectorXd& joint_pos,
																	  const rbd::Vector6d& base_vel,
																	  const Eigen::VectorXd& joint_vel,
																	  const rbd::Vector6d& base_acc,
																	  const Eigen::VectorXd& joint_acc,
																	  const rbd::EndEffectorSelector& contacts)
{
	// Computing the feasible accelerations //TODO figure out about this topic
	rbd::Vector6d base_feas_acc = base_acc;
	Eigen::VectorXd joint_feas_acc = joint_acc;


	// Computing the base wrench assuming a fully actuation on the floating-base
	rbd::Vector6d base_wrench;
	computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_feas_acc, joint_feas_acc);

	// Computing the base contribution of contact jacobian
	Eigen::MatrixXd base_contact_jac, full_jacobian;
	kinematics_.computeJacobian(full_jacobian, base_pos, joint_pos, contacts, dwl::rbd::Linear);
	kinematics_.getFloatingBaseJacobian(base_contact_jac, full_jacobian);


	// Computing the contact forces that generates the desired base wrench with possible physical constraints
	// This approach builds an augmented jacobian matrix as [base contact jacobian; base constraint jacobian]
	rbd::EndEffectorForce contact_forces;
	if (reduced_base_ != NULL && !reduced_base_->isFullyFree()) {
		Eigen::MatrixXd constraint_base_jac = Eigen::MatrixXd::Zero(6,6);
		constraint_base_jac(rbd::TX,rbd::TX) = reduced_base_->TX.active;
		constraint_base_jac(rbd::TY,rbd::TY) = reduced_base_->TY.active;
		constraint_base_jac(rbd::TZ,rbd::TZ) = reduced_base_->TZ.active;
		constraint_base_jac(rbd::RX,rbd::RX) = reduced_base_->TX.active;
		constraint_base_jac(rbd::RY,rbd::RY) = reduced_base_->TY.active;
		constraint_base_jac(rbd::RZ,rbd::RZ) = reduced_base_->TZ.active;

		// Computing the augmented jacobian
		Eigen::MatrixXd augmented_jac = Eigen::MatrixXd::Zero(base_contact_jac.rows() + 6, base_contact_jac.cols());
		augmented_jac.block(0,0,base_contact_jac.rows(), base_contact_jac.cols()) = base_contact_jac;
		augmented_jac.block(base_contact_jac.rows(),0,6,6) = constraint_base_jac;

		// Computing the external forces from the augmented forces [contact forces; base constraint forces]
		Eigen::VectorXd augmented_forces =
				math::pseudoInverse((Eigen::MatrixXd) augmented_jac.transpose()) * base_wrench;
		unsigned int num_active_contacts = contacts.size();
		contact_forces[robot_model_.GetBodyName(6)] << augmented_forces.segment(3 * num_active_contacts + 3, 3),
				augmented_forces.segment(3 * num_active_contacts, 3);
		for (unsigned int i = 0; i < num_active_contacts; i++)
			contact_forces[contacts[i]] << 0., 0., 0., augmented_forces.segment(3*i, 3);
	} else {
		// Computing the external forces from contact forces
		Eigen::VectorXd endeffector_forces =
				math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * base_wrench;
		unsigned int num_active_contacts = contacts.size();
		for (unsigned int i = 0; i < num_active_contacts; i++)
			contact_forces[contacts[i]] << 0., 0., 0., endeffector_forces.segment(3*i, 3);
	}


	// Computing the inverse dynamic algorithm with the desired contact forces, and the base constraint forces for
	// floating-base with physical constraints. This should generate the joint forces with a null base wrench
	computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_feas_acc, joint_feas_acc, contact_forces);
	std::cout << "joint forces = " << joint_forces.transpose() << std::endl;
	std::cout << "base wrench = " << base_wrench.transpose() << std::endl;


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
			rbd::Vector6d force = ext_force.at(body_name);
			Eigen::Vector3d force_point =
					CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
			rbd::Vector6d spatial_force = rbd::convertPointForceToSpatialForce(force, force_point);

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
			rbd::Vector6d force = ext_force.at(body_name);
			Eigen::Vector3d force_point =
					CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
			rbd::Vector6d spatial_force = rbd::convertPointForceToSpatialForce(force, force_point);

			unsigned parent_id = robot_model_.mFixedBodies[it].mMovableParent;
			fext.at(parent_id) += spatial_force;
		}
	}
}

} //@namespace model
} //@namespace dwl
