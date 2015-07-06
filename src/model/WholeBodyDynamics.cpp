#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

WholeBodyDynamics::WholeBodyDynamics() : type_of_system_(rbd::FixedBase), reduced_base_(NULL)
{

}


WholeBodyDynamics::~WholeBodyDynamics()
{

}


void WholeBodyDynamics::modelFromURDFFile(std::string model_file,
										  struct rbd::ReducedFloatingBase* reduced_base,
										  bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);
	reduced_base_ = reduced_base;

	kinematics_.modelFromURDFFile(model_file.c_str(), reduced_base, false);

	// Getting the type of dynamic system
	rbd::getTypeOfDynamicSystem(type_of_system_, robot_model_, reduced_base);

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(robot_model_);
}


void WholeBodyDynamics::modelFromURDFModel(std::string urdf_model,
										   struct rbd::ReducedFloatingBase* reduced_base,
										   bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromString(urdf_model.c_str(), &robot_model_, false);
	reduced_base_ = reduced_base;

	kinematics_.modelFromURDFModel(urdf_model.c_str(), reduced_base, false);

	// Getting the type of dynamic system
	rbd::getTypeOfDynamicSystem(type_of_system_, robot_model_, reduced_base);

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(robot_model_);
}


void WholeBodyDynamics::computeInverseDynamics(rbd::Vector6d& base_wrench,
											   Eigen::VectorXd& joint_forces,
											   const rbd::Vector6d& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const rbd::Vector6d& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const rbd::Vector6d& base_acc,
											   const Eigen::VectorXd& joint_acc,
											   const rbd::BodyWrench& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, type_of_system_, reduced_base_);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(base_vel, joint_vel, type_of_system_, reduced_base_);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(base_acc, joint_acc, type_of_system_, reduced_base_);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau, &fext);

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::fromGeneralizedJointState(base_wrench, joint_forces, tau, type_of_system_, reduced_base_);
}


void WholeBodyDynamics::computeFloatingBaseInverseDynamics(rbd::Vector6d& base_acc,
														   Eigen::VectorXd& joint_forces,
														   const rbd::Vector6d& base_pos,
														   const Eigen::VectorXd& joint_pos,
														   const rbd::Vector6d& base_vel,
														   const Eigen::VectorXd& joint_vel,
														   const Eigen::VectorXd& joint_acc,
														   const rbd::BodyWrench& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, type_of_system_, reduced_base_);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(base_vel, joint_vel, type_of_system_, reduced_base_);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(base_acc, joint_acc, type_of_system_, reduced_base_);
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
	rbd::fromGeneralizedJointState(base_wrench, joint_forces, tau, type_of_system_, reduced_base_);
}


void WholeBodyDynamics::computeConstrainedFloatingBaseInverseDynamics(Eigen::VectorXd& joint_forces,
																	  const rbd::Vector6d& base_pos,
																	  const Eigen::VectorXd& joint_pos,
																	  const rbd::Vector6d& base_vel,
																	  const Eigen::VectorXd& joint_vel,
																	  const rbd::Vector6d& base_acc,
																	  const Eigen::VectorXd& joint_acc,
																	  const rbd::BodySelector& contacts)
{
	// Computing the contact forces that generates the desired base wrench. A floating-base system can be described as
	// floating-base with or without physical constraints or virtual floating-base. Note that with a virtual floating-base
	// we can describe a n-dimensional floating-base, witch n less than 6
	rbd::BodyWrench contact_forces;
	rbd::Vector6d base_feas_acc = base_acc;
	Eigen::VectorXd joint_feas_acc = joint_acc;
	computeContactForces(contact_forces, joint_forces, base_pos, joint_pos,
						 base_vel, joint_vel, base_feas_acc, joint_feas_acc, contacts);

	// Computing the inverse dynamic algorithm with the desired contact forces, and the base constraint forces for
	// floating-base with physical constraints. This should generate the joint forces with a null base wrench
	rbd::Vector6d base_wrench;
	computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_feas_acc, joint_feas_acc, contact_forces);
}


void WholeBodyDynamics::computeContactForces(rbd::BodyWrench& contact_forces,
											 Eigen::VectorXd& joint_forces,
											 const rbd::Vector6d& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const rbd::Vector6d& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 rbd::Vector6d& base_acc,
											 Eigen::VectorXd& joint_acc,
											 const rbd::BodySelector& contacts)
{
	// Computing the fixed-base jacobian and base contact jacobian. These jacobians are used for computing a consistent
	// joint acceleration, and for mapping desired base wrench to joint forces
	Eigen::MatrixXd full_jac;
	kinematics_.computeJacobian(full_jac, base_pos, joint_pos, contacts, rbd::Linear);

	// Computing the consistent joint accelerations given a desired base acceleration and contact definition. We assume
	// that contacts are static, which it allows us to computed a consistent joint accelerations.
	rbd::Vector6d base_feas_acc;
	Eigen::VectorXd joint_feas_acc;
	computeConstrainedConsistentAcceleration(base_feas_acc, joint_feas_acc, base_pos, joint_pos,
											 base_vel, joint_vel, base_acc, joint_acc, contacts);


	// Computing the desired base wrench assuming a fully actuation on the floating-base
	rbd::Vector6d base_wrench;
	computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_feas_acc, joint_feas_acc);

	// Rewriting the base and joint acceleration
	base_acc = base_feas_acc;
	joint_acc = joint_feas_acc;

	// Computing the base contribution of contact jacobian
	Eigen::MatrixXd base_contact_jac;
	kinematics_.getFloatingBaseJacobian(base_contact_jac, full_jac);

	// Computing the contact forces that generates the desired base wrench. A floating-base system can be described as
	// floating-base with or without physical constraints or virtual floating-base. Note that with a virtual floating-base
	// we can describe a n-dimensional floating-base, witch n less than 6
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
}


void WholeBodyDynamics::computeContactForces(rbd::BodyWrench& contact_forces,
											 const rbd::Vector6d& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const rbd::Vector6d& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 const rbd::Vector6d& base_acc,
											 const Eigen::VectorXd& joint_acc,
											 const Eigen::VectorXd& joint_forces,
											 const rbd::BodySelector& contacts)
{
	// Computing the estimated joint forces assuming that there aren't contact forces
	dwl::rbd::Vector6d base_wrench;
	Eigen::VectorXd estimated_joint_forces;
	computeInverseDynamics(base_wrench, estimated_joint_forces, base_pos, joint_pos,
						   base_vel, joint_vel, base_acc, joint_acc);

	// Computing the joint force error
	Eigen::VectorXd joint_force_error = estimated_joint_forces - joint_forces;

	// Computing the contact forces
	for (rbd::BodySelector::const_iterator contact_iter = contacts.begin();
			contact_iter != contacts.end();
			contact_iter++)
	{
		std::string body_name = *contact_iter;
		unsigned int body_id = robot_model_.GetBodyId(body_name.c_str());
		rbd::BodySelector body(contact_iter, contact_iter + 1);

		Eigen::MatrixXd full_jac, fixed_jac;
		kinematics_.computeJacobian(full_jac, base_pos, joint_pos, body, rbd::Linear);
		kinematics_.getFixedBaseJacobian(fixed_jac, full_jac);

		Eigen::Vector3d force = dwl::math::pseudoInverse((Eigen::MatrixXd) fixed_jac.transpose()) *
				(rbd::getBranchState(joint_force_error, body_id, robot_model_, reduced_base_));

		contact_forces[body_name] << 0, 0, 0, force;
	}
}


void WholeBodyDynamics::convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& fext,
													 const rbd::BodyWrench& ext_force,
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


void WholeBodyDynamics::computeConstrainedConsistentAcceleration(rbd::Vector6d& base_feas_acc,
																 Eigen::VectorXd& joint_feas_acc,
																 const rbd::Vector6d& base_pos,
																 const Eigen::VectorXd& joint_pos,
																 const rbd::Vector6d& base_vel,
																 const Eigen::VectorXd& joint_vel,
																 const rbd::Vector6d& base_acc,
																 const Eigen::VectorXd& joint_acc,
																 const rbd::BodySelector& contacts)
{
	// Computing the fixed-base jacobian and base contact jacobian. These jacobians are used for computing a consistent
	// joint acceleration, and for mapping desired base wrench to joint forces
	Eigen::MatrixXd full_jac, fixed_jac;
	kinematics_.computeJacobian(full_jac, base_pos, joint_pos, contacts, rbd::Linear);
	kinematics_.getFixedBaseJacobian(fixed_jac, full_jac);


	// Computing the consistent joint accelerations given a desired base acceleration and contact definition. We assume
	// that contacts are static, which it allows us to computed a consistent joint accelerations.
	base_feas_acc = base_acc;
	joint_feas_acc = joint_acc;

	// At the first step, we compute the angular and linear floating-base velocity and acceleration
	rbd::Vector6d base_des_vel(base_vel);
	Eigen::Vector3d base_ang_vel = rbd::angularFloatingBaseState(base_des_vel);
	Eigen::Vector3d base_lin_vel = rbd::linearFloatingBaseState(base_des_vel);

	rbd::Vector6d base_des_acc(base_acc);
	Eigen::Vector3d base_ang_acc = rbd::angularFloatingBaseState(base_des_acc);
	Eigen::Vector3d base_lin_acc = rbd::linearFloatingBaseState(base_des_acc);

	// Computing contact linear positions and the J_d*q_d component, which are used for computing the joint accelerations
	Eigen::VectorXd op_pos, jacd_qd;
	kinematics_.computeForwardKinematics(op_pos, base_pos, joint_pos, contacts, rbd::Linear);
	kinematics_.computeJdotQdot(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, contacts, rbd::Linear);

	// Computing the consistent joint acceleration given a base state
	for (unsigned int i = 0; i < contacts.size(); i++) {
		Eigen::Vector3d contact_pos = op_pos.segment<3>(i * 3);

		// Computing the desired contact velocity
		Eigen::Vector3d contact_vel = -base_lin_vel - base_ang_vel.cross(contact_pos);
		Eigen::Vector3d contact_acc = -base_lin_acc - base_ang_acc.cross(contact_pos) - base_ang_vel.cross(contact_pos)
				- 2 * base_ang_vel.cross(contact_vel);

		// Computing the join acceleration from x_dd = J*q_dd + J_d*q_d since we are doing computation in the base frame
		Eigen::VectorXd q_dd = math::pseudoInverse(fixed_jac) * (contact_acc - jacd_qd.segment<3>(i * 3));

		unsigned int body_contact_id = robot_model_.GetBodyId(contacts[i].c_str());
		rbd::setBranchState(joint_feas_acc, q_dd, body_contact_id, robot_model_, reduced_base_);
	}
}

} //@namespace model
} //@namespace dwl
