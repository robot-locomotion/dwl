#include <dwl/model/WholeBodyDynamics.h>


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


void WholeBodyDynamics::modelFromURDFFile(const std::string& urdf_file,
										  const std::string& system_file,
										  bool info)
{
	modelFromURDFModel(urdf_model::fileToXml(urdf_file), system_file, info);
}


void WholeBodyDynamics::modelFromURDFModel(const std::string& urdf_model,
										   const std::string& system_file,
										   bool info)
{
	// Reseting the floating-base system information given an URDF model
	system_.resetFromURDFModel(urdf_model, system_file);

	// Setting the kinematic model from URDF model
	kinematics_.modelFromURDFModel(urdf_model.c_str(), system_file, false);

	// Getting the list of movable and fixed bodies
	rbd::getListOfBodies(body_id_, system_.getRBDModel());

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(system_.getRBDModel());

	// Setting up the size of the joint space inertia matrix
	joint_inertia_mat_.resize(system_.getSystemDoF(), system_.getSystemDoF());
	joint_inertia_mat_.setZero();
}


void WholeBodyDynamics::computeInverseDynamics(rbd::Vector6d& base_wrench,
											   Eigen::VectorXd& joint_forces,
											   const rbd::Vector6d& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const rbd::Vector6d& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const rbd::Vector6d& base_acc,
											   const Eigen::VectorXd& joint_acc,
											   const rbd::BodyVector6d& ext_force)
{
	// Setting the size of the joint forces vector
	joint_forces.resize(system_.getJointDoF());

	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);
	Eigen::VectorXd q_dot = system_.toGeneralizedJointState(base_vel, joint_vel);
	Eigen::VectorXd q_ddot = system_.toGeneralizedJointState(base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system_.getSystemDoF());

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::InverseDynamics(system_.getRBDModel(), q, q_dot, q_ddot, tau, &fext);

	// Converting the generalized joint forces to base wrench and joint forces
	base_wrench.setZero();
	system_.fromGeneralizedJointState(base_wrench, joint_forces, tau);
}


void WholeBodyDynamics::computeFloatingBaseInverseDynamics(rbd::Vector6d& base_acc,
														   Eigen::VectorXd& joint_forces,
														   const rbd::Vector6d& base_pos,
														   const Eigen::VectorXd& joint_pos,
														   const rbd::Vector6d& base_vel,
														   const Eigen::VectorXd& joint_vel,
														   const Eigen::VectorXd& joint_acc,
														   const rbd::BodyVector6d& ext_force)
{//TODO test floating-base ID, and develops the virtual floating-base ID (general hybrid dynamics?)
	// Setting the size of the joint forces vector
	joint_forces.resize(system_.getJointDoF());

	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);
	Eigen::VectorXd q_dot = system_.toGeneralizedJointState(base_vel, joint_vel);
	Eigen::VectorXd q_ddot = system_.toGeneralizedJointState(base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system_.getSystemDoF());

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	if (system_.isFullyFloatingBase()) {
		RigidBodyDynamics::Math::SpatialVector base_ddot =
				RigidBodyDynamics::Math::SpatialVector(base_acc);
		rbd::FloatingBaseInverseDynamics(system_.getRBDModel(),
										 q, q_dot, q_ddot, base_ddot, tau, &fext);

		// Converting the base acceleration
		base_acc = base_ddot;
	} else {
		if (system_.isVirtualFloatingBaseRobot()) {
			RigidBodyDynamics::Math::SpatialVector base_ddot =
					RigidBodyDynamics::Math::SpatialVector(base_acc);
			rbd::FloatingBaseInverseDynamics(system_.getRBDModel(), 1,
											q, q_dot, q_ddot, base_ddot,
											tau, &fext);
			base_acc = base_ddot;
//			RigidBodyDynamics::InverseDynamics(system_.getRBDModel(), q, q_dot, q_ddot, tau, &fext);
//			tau(0) = 0;
//			RigidBodyDynamics::ForwardDynamics(system_.getRBDModel(), q, q_dot, tau, q_ddot, &fext);
//			base_acc(rbd::LZ) = q_ddot(0);
		} else
			printf(YELLOW "WARNING: this is not a floating-base system\n" COLOR_RESET);
	}

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::Vector6d base_wrench;
	system_.fromGeneralizedJointState(base_wrench, joint_forces, tau);
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
	// Setting the size of the joint forces vector
	joint_forces.resize(system_.getJointDoF());

	// Computing the contact forces that generates the desired base wrench. A
	// floating-base system can be described as floating-base with or without
	// physical constraints or virtual floating-base. Note that with a virtual
	// floating-base we can describe a n-dimensional floating-base, which n
	// less than 6
	rbd::BodyVector6d contact_forces;
	rbd::Vector6d base_feas_acc = base_acc;
	Eigen::VectorXd joint_feas_acc = joint_acc;
	computeContactForces(contact_forces, joint_forces,
						 base_pos, joint_pos,
						 base_vel, joint_vel,
						 base_feas_acc, joint_feas_acc,
						 contacts);

	// Computing the inverse dynamic algorithm with the desired contact forces,
	// and the base constraint forces for floating-base with physical
	// constraints. This should generate the joint forces with a null base wrench
	rbd::Vector6d base_wrench;
	computeInverseDynamics(base_wrench, joint_forces,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_feas_acc, joint_feas_acc,
						   contact_forces);
}


const Eigen::MatrixXd& WholeBodyDynamics::computeJointSpaceInertiaMatrix(const rbd::Vector6d& base_pos,
																		 const Eigen::VectorXd& joint_pos)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);

	// Computing the joint space inertia matrix using the Composite
	// Rigid Body Algorithm
	RigidBodyDynamics::CompositeRigidBodyAlgorithm(system_.getRBDModel(),
												   q, joint_inertia_mat_, true);

	// Changing the floating-base inertia matrix component to the order
	// [Angular, Linear]
	if (system_.isFullyFloatingBase()) {
		Eigen::MatrixXd base_lin_mat = joint_inertia_mat_.block<3,6>(0,0);
		Eigen::MatrixXd base_ang_mat = joint_inertia_mat_.block<3,6>(3,0);

		// Writing the new order
		joint_inertia_mat_.block<3,6>(rbd::AX, 0) << base_ang_mat.rightCols(3),
				base_ang_mat.leftCols(3);
		joint_inertia_mat_.block<3,6>(rbd::LX, 0) << base_lin_mat.rightCols(3),
				base_lin_mat.leftCols(3);
	}

	return joint_inertia_mat_;
}


const rbd::Matrix6d& WholeBodyDynamics::computeCentroidalInertiaMatrix(const rbd::Vector6d& base_pos,
																	   const Eigen::VectorXd& joint_pos)
{
	// We compute the centroidal inertia matrix from the joint-space inertia
	// matrix, i.e. as I_com = base_X_com^T * Ic * base_X_com
	// Getting the joint-space inertia matrix
	Eigen::MatrixXd I_base = computeJointSpaceInertiaMatrix(base_pos, joint_pos);
	
	// Getting the spatial transform from CoM to base frame
	Eigen::Vector3d com_pos = system_.getSystemCoM(base_pos, joint_pos);
	RigidBodyDynamics::Math::SpatialTransform base_X_com(Eigen::Matrix3d::Identity(), -com_pos);
	com_inertia_mat_ = base_X_com.toMatrixTranspose() * I_base * base_X_com.toMatrix();

	return com_inertia_mat_;
}


void WholeBodyDynamics::computeContactForces(rbd::BodyVector6d& contact_forces,
											 Eigen::VectorXd& joint_forces,
											 const rbd::Vector6d& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const rbd::Vector6d& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 rbd::Vector6d& base_acc,
											 Eigen::VectorXd& joint_acc,
											 const rbd::BodySelector& contacts)
{
	// Setting the size of the joint forces vector
	joint_forces.resize(system_.getJointDoF());

	// Computing the fixed-base jacobian and base contact jacobian. These
	// jacobians are used for computing a consistent joint acceleration, and
	// for mapping desired base wrench to joint forces
	Eigen::MatrixXd full_jac;
	kinematics_.computeJacobian(full_jac,
								base_pos, joint_pos,
								contacts, rbd::Linear);

	// Computing the consistent joint accelerations given a desired base
	// acceleration and contact definition. We assume that contacts are static,
	// which it allows us to computed a consistent joint accelerations.
	rbd::Vector6d base_feas_acc = rbd::Vector6d::Zero();
	Eigen::VectorXd joint_feas_acc(system_.getJointDoF());
	computeConstrainedConsistentAcceleration(base_feas_acc, joint_feas_acc,
											 base_pos, joint_pos,
											 base_vel, joint_vel,
											 base_acc, joint_acc,
											 contacts);

	// Computing the desired base wrench assuming a fully actuation on the
	// floating-base
	rbd::Vector6d base_wrench;
	computeInverseDynamics(base_wrench, joint_forces,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_feas_acc, joint_feas_acc);

	// Rewriting the base and joint acceleration
	base_acc = base_feas_acc;
	joint_acc = joint_feas_acc;

	// Computing the base contribution of contact jacobian
	Eigen::MatrixXd base_contact_jac;
	kinematics_.getFloatingBaseJacobian(base_contact_jac, full_jac);

	// Computing the contact forces that generates the desired base wrench. A
	// floating-base system can be described as floating-base with or without
	// physical constraints or virtual floating-base. Note that with a virtual
	// floating-base we can describe a n-dimensional floating-base, witch n
	// less than 6
	if (system_.isFullyFloatingBase()) {
		// This approach builds an augmented jacobian matrix as [base contact
		// jacobian; base constraint jacobian]. Therefore, we compute
		// constrained reaction forces in the base.
		if (system_.isConstrainedFloatingBaseRobot()) {
			// Computing the base constraint contribution to the jacobian
			Eigen::MatrixXd base_constraint_jac = Eigen::MatrixXd::Zero(6,6);
			for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
				rbd::Coords6d base_coord = rbd::Coords6d(base_idx);
				FloatingBaseJoint base_joint =
						system_.getFloatingBaseJoint(base_coord);

				base_constraint_jac(base_coord, base_coord) = !base_joint.constrained;
			}

			// Computing the augmented jacobian
			// [base contact jacobian; base constraint jacobian]
			Eigen::MatrixXd augmented_jac =
					Eigen::MatrixXd::Zero(base_contact_jac.rows() + 6,
										  base_contact_jac.cols());
			augmented_jac.block(0, 0, base_contact_jac.rows(), base_contact_jac.cols()) = base_contact_jac;
			augmented_jac.block<6,6>(base_contact_jac.rows(), 0) = base_constraint_jac;

			// Computing the external forces from the augmented forces
			// [contact forces; base constraint forces]
			Eigen::VectorXd augmented_forces =
					math::pseudoInverse((Eigen::MatrixXd) augmented_jac.transpose()) * base_wrench;

			// Adding the base reaction forces in the set of external forces
			contact_forces[system_.getRBDModel().GetBodyName(6)] =
					augmented_forces.tail<6>();

			// Adding the contact forces in the set of external forces
			unsigned int num_active_contacts = contacts.size();
			for (unsigned int i = 0; i < num_active_contacts; i++)
				contact_forces[contacts[i]] << 0., 0., 0., augmented_forces.segment<3>(3 * i);
		} else {
			// This is a floating-base without physical constraints. So, we
			// don't need to augment the jacobian
			// Computing the external forces from contact forces
			Eigen::VectorXd endeffector_forces =
					math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * base_wrench;

			// Adding the contact forces in the set of external forces
			unsigned int num_active_contacts = contacts.size();
			for (unsigned int i = 0; i < num_active_contacts; i++)
				contact_forces[contacts[i]] << 0., 0., 0., endeffector_forces.segment<3>(3 * i);
		}
	} else if (system_.isVirtualFloatingBaseRobot()) {
		// This is n-dimensional floating-base system. So, we need to compute
		// a virtual base wrench
		Eigen::VectorXd virtual_base_wrench =
				Eigen::VectorXd::Zero(system_.getFloatingBaseDoF());
		for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
			rbd::Coords6d base_coord = rbd::Coords6d(base_idx);
			FloatingBaseJoint base_joint = system_.getFloatingBaseJoint(base_coord);

			if (base_joint.active)
				virtual_base_wrench(base_joint.id) = base_wrench(base_coord);
		}

		// Computing the contact forces that generates the desired base wrench
		// in the case of n dof floating-base, where n is less than 6. Note
		// that we describe this floating-base as an under-actuated virtual
		// floating-base joints
		Eigen::VectorXd endeffector_forces =
				math::pseudoInverse((Eigen::MatrixXd) base_contact_jac.transpose()) * virtual_base_wrench;

		// Adding the contact forces in the set of external forces
		unsigned int num_active_contacts = contacts.size();
		for (unsigned int i = 0; i < num_active_contacts; i++)
			contact_forces[contacts[i]] << 0., 0., 0., endeffector_forces.segment<3>(3 * i);
	}
}


void WholeBodyDynamics::estimateContactForces(rbd::BodyVector6d& contact_forces,
											 const rbd::Vector6d& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const rbd::Vector6d& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 const rbd::Vector6d& base_acc,
											 const Eigen::VectorXd& joint_acc,
											 const Eigen::VectorXd& joint_forces,
											 const rbd::BodySelector& contacts)
{
	// Computing the estimated joint forces assuming that there aren't
	// contact forces
	dwl::rbd::Vector6d base_wrench;
	Eigen::VectorXd estimated_joint_forces;
	computeInverseDynamics(base_wrench, estimated_joint_forces,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_acc, joint_acc);

	// Computing the joint force error
	Eigen::VectorXd joint_force_error = estimated_joint_forces - joint_forces;

	// Computing the contact forces
	for (rbd::BodySelector::const_iterator contact_iter = contacts.begin();
			contact_iter != contacts.end();
			contact_iter++)
	{
		std::string body_name = *contact_iter;
		rbd::BodySelector body(contact_iter, contact_iter + 1);

		Eigen::MatrixXd fixed_jac;
		kinematics_.computeFixedJacobian(fixed_jac,
										 joint_pos,
										 body_name, rbd::Linear);

		Eigen::Vector3d force =
				math::pseudoInverse((Eigen::MatrixXd) fixed_jac.transpose()) *
				system_.getBranchState(joint_force_error, body_name);

		contact_forces[body_name] << 0, 0, 0, force;
	}
}


void WholeBodyDynamics::computeCenterOfPressure(Eigen::Vector3d& cop_pos,
												const rbd::BodyVector6d& contact_for,
												const rbd::BodyVectorXd& contact_pos)
{
	// TODO: Compute the CoM position for case when the normal surface is different to z
	// Initializing the variables
	cop_pos.setZero();
	double sum = 0.;

	// Getting the names of the feet
	rbd::BodySelector ground_contacts = system_.getEndEffectorNames(model::FOOT);

	// Sanity check: checking if there are contact information and the size
	if ((contact_for.size() == 0) || (contact_pos.size() == 0) ||
			contact_for.size() != contact_pos.size()) {
		printf(YELLOW "Warning: could not compute the CoP because there is"
				" missing information\n" COLOR_RESET);
		return;
	}

	// Computing the Center of Pressure (CoP) position
	for (rbd::BodySelector::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); contact_iter++) {
		std::string name = *contact_iter;

		// Getting the ground reaction forces
		rbd::Vector6d force;
		rbd::BodyVector6d::const_iterator for_it = contact_for.find(name);
		if (for_it != contact_for.end())
			force = for_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact force of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		// Getting the contact position
		rbd::BodyVectorXd::const_iterator pos_it = contact_pos.find(name);
		Eigen::VectorXd position;
		if (pos_it != contact_pos.end())
			position = pos_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact position of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		// Accumulate the cop position as a weighted sum, where the weight is
		// the z component of the force at each contact point
		double norm_for = force(rbd::LZ);
		if (position.size() == 6)
			cop_pos += norm_for * position.segment<3>(rbd::LX);
		else
			cop_pos += norm_for * position;
		sum += norm_for;
	}
	cop_pos /= sum;
}


void WholeBodyDynamics::computeInstantaneousCapturePoint(Eigen::Vector3d& icp_pos,
														 const Eigen::Vector3d& com_pos,
		                                                 const Eigen::Vector3d& com_vel,
		                                                 const double& height)
{
	if (height < 0.) {
		printf(YELLOW "Warning: the height should be a positive value\n" COLOR_RESET);
	}

	double omega = sqrt(system_.getGravityAcceleration() / height);
	icp_pos = com_pos + com_vel / omega;
	icp_pos(rbd::Z) = com_pos(rbd::Z) - height;
}


void WholeBodyDynamics::computeCentroidalMomentPivot(Eigen::Vector3d& cmp_pos,
													 const Eigen::Vector3d& com_pos,
													 const double& height,
													 const rbd::BodyVector6d& contact_for)
{
	if (height < 0.) {
		printf(YELLOW "Warning: the height should be a positive value\n" COLOR_RESET);
	}

	// Getting the names of the feet
	rbd::BodySelector ground_contacts = system_.getEndEffectorNames(model::FOOT);

	// The Centroidal Momentum Pivot (CMP) is computed given the GRFs
	double grf_x = 0., grf_y = 0., grf_z = 0.;
	for (rbd::BodySelector::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); contact_iter++) {
		std::string name = *contact_iter;

		// Getting the ground reaction forces
		rbd::Vector6d force;
		rbd::BodyVector6d::const_iterator for_it = contact_for.find(name);
		if (for_it != contact_for.end())
			force = for_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact force of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		grf_x += force(rbd::LX);
		grf_y += force(rbd::LY);
		grf_z += force(rbd::LZ);
	}

	cmp_pos(rbd::X) =
			com_pos(rbd::X) - grf_x / grf_z * height;
	cmp_pos(rbd::Y) =
			com_pos(rbd::Y) - grf_y / grf_z * height;
	cmp_pos(rbd::Z) = com_pos(rbd::Z) - height;
}


void WholeBodyDynamics::computeCoMTorque(Eigen::Vector3d& torque,
										 const Eigen::Vector3d& cop_pos,
										 const Eigen::Vector3d& cmp_pos,
										 const rbd::BodyVector6d& contact_for)
{
	// Getting the names of the feet
	rbd::BodySelector ground_contacts = system_.getEndEffectorNames(model::FOOT);

	// The Centroidal Momentum Pivot (CMP) is computed given the GRFs
	double grf_z = 0.;
	for (rbd::BodySelector::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); contact_iter++) {
		std::string name = *contact_iter;

		// Getting the ground reaction forces
		rbd::Vector6d force;
		rbd::BodyVector6d::const_iterator for_it = contact_for.find(name);
		if (for_it != contact_for.end())
			force = for_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact force of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}
		grf_z += force(rbd::LZ);
	}

	torque(rbd::X) = grf_z * (cop_pos(rbd::Y) - cmp_pos(rbd::Y));
	torque(rbd::Y) = grf_z * (cmp_pos(rbd::X) - cop_pos(rbd::X));
	torque(rbd::Z) = 0.;
}


void WholeBodyDynamics::estimateGroundReactionForces(rbd::BodyVector6d& contact_for,
													 const Eigen::Vector3d& cop_pos,
													 const rbd::BodyVector3d& contact_pos,
													 const rbd::BodySelector& ground_contacts)
{
	// Sanity check: checking if there are contact information and the size
	if (contact_pos.size() == 0) {
		printf(YELLOW "Warning: could not compute the CoP because there is"
				" missing information" COLOR_RESET);
		return;
	}

	// Computing the Center of Pressure (CoP) position
	unsigned int idx = 0;
	unsigned int num_contacts = ground_contacts.size();
	Eigen::MatrixXd contact_mat = Eigen::MatrixXd::Zero(3, num_contacts);
	for (rbd::BodySelector::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); contact_iter++) {
		std::string name = *contact_iter;

		// Getting the contact position
		rbd::BodyVector3d::const_iterator pos_it = contact_pos.find(name);
		Eigen::VectorXd position;
		if (pos_it != contact_pos.end())
			position = pos_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact position of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		// Filling the contact position matrix
		contact_mat.col(idx) = position;

		idx++;
	}

	// Computing the normal contact forces
	double weight = system_.getTotalMass() * system_.getGravityAcceleration();
	Eigen::VectorXd norm_for = math::pseudoInverse(contact_mat) * cop_pos * weight;

	// Filling the contact forces vector
	idx = 0;
	for (rbd::BodySelector::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); contact_iter++) {
		std::string name = *contact_iter;

		contact_for[name] << 0, 0, 0, 0, 0, norm_for[idx];
		idx++;
	}
}


void WholeBodyDynamics::estimateActiveContactsAndForces(rbd::BodySelector& active_contacts,
														rbd::BodyVector6d& contact_forces,
														const rbd::Vector6d& base_pos,
														const Eigen::VectorXd& joint_pos,
														const rbd::Vector6d& base_vel,
														const Eigen::VectorXd& joint_vel,
														const rbd::Vector6d& base_acc,
														const Eigen::VectorXd& joint_acc,
														const Eigen::VectorXd& joint_forces,
														const rbd::BodySelector& contacts,
														double force_threshold)
{
	// Computing the contact forces for a predefined set of end-effector
	estimateContactForces(contact_forces,
						 base_pos, joint_pos,
						 base_vel, joint_vel,
						 base_acc, joint_acc,
						 joint_forces, contacts);

	// Detecting active end-effector by using a force threshold
	getActiveContacts(active_contacts,
					  contact_forces,
					  force_threshold);
}


void WholeBodyDynamics::estimateActiveContacts(rbd::BodySelector& active_contacts,
											   const rbd::Vector6d& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const rbd::Vector6d& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const rbd::Vector6d& base_acc,
											   const Eigen::VectorXd& joint_acc,
											   const Eigen::VectorXd& joint_forces,
											   const rbd::BodySelector& contacts,
											   double force_threshold)
{
	// Computing the contact forces in predefined set of end-effector
	rbd::BodyVector6d contact_forces;
	estimateActiveContactsAndForces(active_contacts,
						   contact_forces,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_acc, joint_acc,
						   joint_forces, contacts,
						   force_threshold);
}


const FloatingBaseSystem& WholeBodyDynamics::getFloatingBaseSystem() const
{
	return system_;
}


const WholeBodyKinematics& WholeBodyDynamics::getWholeBodyKinematics() const
{
	return kinematics_;
}


void WholeBodyDynamics::getActiveContacts(rbd::BodySelector& active_contacts,
										  const rbd::BodyVector6d& contact_forces,
										  double force_threshold)
{
	// Detecting active end-effector by using a force threshold
	for (rbd::BodyVector6d::const_iterator endeffector_it = contact_forces.begin();
			endeffector_it != contact_forces.end(); endeffector_it++) {
		std::string endeffector_name = endeffector_it->first;
		dwl::rbd::Vector6d contact_wrench = endeffector_it->second;

		if (contact_wrench.norm() > force_threshold)
			active_contacts.push_back(endeffector_name);
	}
}


void WholeBodyDynamics::convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& fext,
													 const rbd::BodyVector6d& ext_force,
													 const Eigen::VectorXd& q)
{
	// Computing the applied external spatial forces for every body
	fext.resize(system_.getRBDModel().mBodies.size());
	// Searching over the movable bodies
	for (unsigned int body_id = 0;
			body_id < system_.getRBDModel().mBodies.size(); body_id++) {
		std::string body_name = system_.getRBDModel().GetBodyName(body_id);

		if (ext_force.count(body_name) > 0) {
			// Converting the applied force to spatial force vector in
			// base coordinates
			rbd::Vector6d force = ext_force.at(body_name);
			Eigen::Vector3d force_point =
					CalcBodyToBaseCoordinates(system_.getRBDModel(),
											  q, body_id,
											  Eigen::Vector3d::Zero(), true);
			rbd::Vector6d spatial_force =
					rbd::convertPointForceToSpatialForce(force, force_point);

			fext.at(body_id) = spatial_force;
		} else
			fext.at(body_id) = rbd::Vector6d::Zero();
	}

	// Searching over the fixed bodies
	for (unsigned int it = 0; it < system_.getRBDModel().mFixedBodies.size(); it++) {
		unsigned int body_id = it + system_.getRBDModel().fixed_body_discriminator;
		std::string body_name = system_.getRBDModel().GetBodyName(body_id);

		if (ext_force.count(body_name) > 0) {
			// Converting the applied force to spatial force vector in
			// base coordinates
			rbd::Vector6d force = ext_force.at(body_name);
			Eigen::Vector3d force_point =
					CalcBodyToBaseCoordinates(system_.getRBDModel(),
											  q, body_id,
											  Eigen::Vector3d::Zero(), true);
			rbd::Vector6d spatial_force =
					rbd::convertPointForceToSpatialForce(force, force_point);

			unsigned parent_id = system_.getRBDModel().mFixedBodies[it].mMovableParent;
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
	// Computing the consistent joint accelerations given a desired base
	// acceleration and contact definition. We assume that contacts are static,
	// which it allows us to computed a consistent joint accelerations.
	base_feas_acc = base_acc;
	joint_feas_acc = joint_acc;

	// At the first step, we compute the angular and linear floating-base
	// velocity and acceleration
	rbd::Vector6d base_des_vel(base_vel);
	Eigen::Vector3d base_ang_vel = rbd::angularPart(base_des_vel);
	Eigen::Vector3d base_lin_vel = rbd::linearPart(base_des_vel);

	rbd::Vector6d base_des_acc(base_acc);
	Eigen::Vector3d base_ang_acc = rbd::angularPart(base_des_acc);
	Eigen::Vector3d base_lin_acc = rbd::linearPart(base_des_acc);

	// Computing contact linear positions and the J_d*q_d component, which
	// are used for computing the joint accelerations
	rbd::BodyVectorXd op_pos, jacd_qd;
	kinematics_.computeForwardKinematics(op_pos,
										 base_pos, joint_pos,
										 contacts, rbd::Linear);
	kinematics_.computeJdotQdot(jacd_qd,
								base_pos, joint_pos,
								base_vel, joint_vel,
								contacts, rbd::Linear);

	// Computing the consistent joint acceleration given a base state
	for (rbd::BodySelector::const_iterator contact_iter = contacts.begin();
			contact_iter != contacts.end();
			contact_iter++)
	{
		std::string contact_name = *contact_iter;
		if (body_id_.count(contact_name) > 0) {
			Eigen::Vector3d contact_pos = op_pos[contact_name];

			// Computing the desired contact velocity
			Eigen::Vector3d contact_vel =
					-base_lin_vel - base_ang_vel.cross(contact_pos);
			Eigen::Vector3d contact_acc =
					-base_lin_acc - base_ang_acc.cross(contact_pos) -
					base_ang_vel.cross(contact_pos) - 2 * base_ang_vel.cross(contact_vel);

			// Computing the fixed-base jacobian
			Eigen::MatrixXd fixed_jac;
			kinematics_.computeFixedJacobian(fixed_jac,
											 joint_pos,
											 contact_name, rbd::Linear);

			// Computing the join acceleration from x_dd = J*q_dd + J_d*q_d
			// since we are doing computation in the base frame
			Eigen::VectorXd q_dd =
					math::pseudoInverse(fixed_jac) * (contact_acc - jacd_qd[contact_name]);

			// Setting up the branch joint acceleration
			system_.setBranchState(joint_feas_acc, q_dd, contact_name);
		}
	}
}

} //@namespace model
} //@namespace dwl
