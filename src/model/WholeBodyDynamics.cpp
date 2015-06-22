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
	if (rbd::isFloatingBaseRobot(robot_model_)) {
		RigidBodyDynamics::Math::SpatialVector base_ddot = RigidBodyDynamics::Math::SpatialVector(base_acc);
		rbd::FloatingBaseInverseDynamics(robot_model_, q, q_dot, q_ddot, base_ddot, tau, &fext);

		// Converting the base acceleration
		base_acc = base_ddot;
	} else {
		RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau, &fext);
		if (!rbd::isVirtualFloatingBaseRobot(reduced_base_))
			printf(YELLOW "Warning: this is not a floating-base system\n" COLOR_RESET);
	}

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::Vector6d base_wrench;
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau, reduced_base_);
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


	// Computing the contact forces that generates the desired base wrench. A floating-base system can be described as
	// floating-base with or without physical constraints or virtual floating-base. Note that with a virtual floating-base
	// we can describe a n-dimensional floating-base, witch n less than 6
	rbd::EndEffectorForce contact_forces;
	if (rbd::isFloatingBaseRobot(robot_model_)) {
		// This approach builds an augmented jacobian matrix as [base contact jacobian; base constraint jacobian].
		// Therefore, we computed constrained reaction forces in the base.
		if (rbd::isConstrainedFloatingBaseRobot(reduced_base_)) {
			// Computing the base constraint contribution to the jacobian
			Eigen::MatrixXd base_constraint_jac = Eigen::MatrixXd::Zero(6,6);
			base_constraint_jac(rbd::TX,rbd::TX) = !reduced_base_->TX.active;
			base_constraint_jac(rbd::TY,rbd::TY) = !reduced_base_->TY.active;
			base_constraint_jac(rbd::TZ,rbd::TZ) = !reduced_base_->TZ.active;
			base_constraint_jac(rbd::RX,rbd::RX) = !reduced_base_->RX.active;
			base_constraint_jac(rbd::RY,rbd::RY) = !reduced_base_->RY.active;
			base_constraint_jac(rbd::RZ,rbd::RZ) = !reduced_base_->RZ.active;

			// Computing the augmented jacobian [base contact jacobian; base constraint jacobian]
			Eigen::MatrixXd augmented_jac = Eigen::MatrixXd::Zero(base_contact_jac.rows() + 6, base_contact_jac.cols());
			augmented_jac.block(0,0,base_contact_jac.rows(), base_contact_jac.cols()) = base_contact_jac;
			augmented_jac.block<6,6>(base_contact_jac.rows(),0) = base_constraint_jac;

			// Computing the external forces from the augmented forces [contact forces; base constraint forces]
			Eigen::VectorXd augmented_forces =
					math::pseudoInverse((Eigen::MatrixXd) augmented_jac.transpose()) * base_wrench;

			// Adding the base reaction forces in the set of external forces
			unsigned int num_active_contacts = contacts.size();
			contact_forces[robot_model_.GetBodyName(6)] << augmented_forces.segment<3>(3 * num_active_contacts + 3),
					augmented_forces.segment<3>(3 * num_active_contacts);

			// Adding the contact forces in the set of external forces
			for (unsigned int i = 0; i < num_active_contacts; i++)
				contact_forces[contacts[i]] << 0., 0., 0., augmented_forces.segment<3>(3 * i);
		} else {
			// This is a floating-base without physical constraints. So, we don't need to augment the jacobian
			// Computing the external forces from contact forces
			Eigen::VectorXd endeffector_forces =
					math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * base_wrench;

			// Adding the contact forces in the set of external forces
			unsigned int num_active_contacts = contacts.size();
			for (unsigned int i = 0; i < num_active_contacts; i++)
				contact_forces[contacts[i]] << 0., 0., 0., endeffector_forces.segment<3>(3 * i);
		}
	} else if (rbd::isVirtualFloatingBaseRobot(reduced_base_)) {
		// This is n-dimensional floating-base system. So, we need to compute a virtual base wrench
		Eigen::VectorXd virtual_base_wrench = Eigen::VectorXd::Zero(reduced_base_->getFloatingBaseDOF());
		if (reduced_base_->TX.active)
			virtual_base_wrench(reduced_base_->TX.id) = base_wrench(rbd::TX);
		if (reduced_base_->TY.active)
			virtual_base_wrench(reduced_base_->TY.id) = base_wrench(rbd::TY);
		if (reduced_base_->TZ.active)
			virtual_base_wrench(reduced_base_->TZ.id) = base_wrench(rbd::TZ);
		if (reduced_base_->RX.active)
			virtual_base_wrench(reduced_base_->RX.id) = base_wrench(rbd::RX);
		if (reduced_base_->RY.active)
			virtual_base_wrench(reduced_base_->RY.id) = base_wrench(rbd::RY);
		if (reduced_base_->RZ.active)
			virtual_base_wrench(reduced_base_->RZ.id) = base_wrench(rbd::RZ);

		// Computing the contact forces that generates the desired base wrench in the case of n dof floating-base, where
		// n is less than 6. Note that we describe this floating-base as an unactuated virtual floating-base joints
		Eigen::VectorXd endeffector_forces =
				math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * virtual_base_wrench;

		// Adding the contact forces in the set of external forces
		unsigned int num_active_contacts = contacts.size();
		for (unsigned int i = 0; i < num_active_contacts; i++)
			contact_forces[contacts[i]] << 0., 0., 0., endeffector_forces.segment<3>(3 * i);
	}

	// Computing the inverse dynamic algorithm with the desired contact forces, and the base constraint forces for
	// floating-base with physical constraints. This should generate the joint forces with a null base wrench
	computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_feas_acc, joint_feas_acc, contact_forces);
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
