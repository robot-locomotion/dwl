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


void WholeBodyDynamics::reset(FloatingBaseSystem& fbs,
							  WholeBodyKinematics& wkin)
{
	// Creating the floating-base system and whole-body kinematics shared pointers
	fbs_ = std::make_shared<FloatingBaseSystem>(fbs);
	wkin_ = std::make_shared<WholeBodyKinematics>(wkin);

	// Setting the kinematic model from URDF model
	wkin_->reset(fbs);

	// Setting the body forces vector
	body_forces_.resize(fbs_->getModel().njoints);
}


void WholeBodyDynamics::computeInverseDynamics(dwl::Force& base_wrench,
											   Eigen::VectorXd& joint_forces,
											   const dwl::SE3& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const dwl::Motion& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const dwl::Motion& base_acc,
											   const Eigen::VectorXd& joint_acc,
											   const dwl::ForceMap& ext_force)
{
	// Checks vector dimensions
	assert(joint_pos.size() == fbs_->getJointDoF());
	assert(joint_vel.size() == fbs_->getJointDoF());
	assert(joint_acc.size() == fbs_->getJointDoF());

	// Converting base and joint states to generalized joint states
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	const Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	const Eigen::VectorXd qdd = fbs_->toTangentState(base_acc, joint_acc);

	// Computing the applied external spatial forces for every body
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q, qd, qdd);
	convertAppliedExternalForces(body_forces_, ext_force);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	se3::rnea(fbs_->getModel(), fbs_->getData(), q, qd, qdd, body_forces_);

	// Converting the generalized joint forces to base wrench and joint forces
	fbs_->fromCotangentState(base_wrench, joint_forces, fbs_->getData().tau);
}


void WholeBodyDynamics::computeConstrainedInverseDynamics(Eigen::VectorXd& joint_forces,
														  Eigen::VectorXd& joint_acc,
														  dwl::ForceMap& contact_forces,
														  const dwl::SE3& base_pos,
														  const Eigen::VectorXd& joint_pos,
														  const dwl::Motion& base_vel,
														  const Eigen::VectorXd& joint_vel,
														  const dwl::Motion& base_acc,
														  const ElementList& contacts)
{
	// Checks vector dimensions
	assert(joint_forces.size() == fbs_->getJointDoF());
	assert(joint_acc.size() == fbs_->getJointDoF());

	// Computing the contact forces that generates the desired base wrench and
	// a set of constrained contacts which are imposes as holonomic constraints
	computeContactForces(contact_forces, joint_acc,
						 base_pos, joint_pos,
						 base_vel, joint_vel,
						 base_acc, contacts);

	// Computing the inverse dynamic algorithm with the desired contact forces,
	// and the base constraint forces for floating-base with physical
	// constraints. This should generate the joint forces with a null base wrench
	dwl::Force base_wrench;
	computeInverseDynamics(base_wrench, joint_forces,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_acc, joint_acc,
						   contact_forces);
}


void WholeBodyDynamics::computeContactForces(dwl::ForceMap& contact_forces,
											 Eigen::VectorXd& joint_acc,
											 const dwl::SE3& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const dwl::Motion& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 const dwl::Motion& base_acc,
											 const ElementList& contacts)
{
	// Checking that it is a floating-base robot
	assert(fbs_->isFixedBase() == false);

	// Checking the dimension of the joint acceleration vector
	assert(joint_acc.size() == fbs_->getJointDoF());

	// Computing the consistent joint accelerations given a desired base
	// acceleration and contact definition. We assume that contacts are static,
	// which it allows us to computed a consistent joint accelerations.
	wkin_->computeConstrainedJointAcceleration(joint_acc,
											   base_pos, joint_pos,
											   base_vel, joint_vel,
											   base_acc, contacts);

	// Computing the desired base wrench assuming a fully actuation on the
	// floating-base
	dwl::Force base_wrench;
	Eigen::VectorXd joint_eff = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	computeInverseDynamics(base_wrench, joint_eff,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_acc, joint_acc);

	// Computing the floating-base Jacobian matrix of all active branches
	Eigen::Matrix6d base_jac;
	unsigned int n_contacts = contacts.size();
	Eigen::Matrix6x A(6, 6*n_contacts);
	A.setZero();
	unsigned int idx = 0;
	for (ElementList::const_iterator it = contacts.begin();
			it != contacts.end(); ++it) {
		// Get the branch properties
		std::string name = *it;
		unsigned int pos_idx, n_dof;
		fbs_->getBranch(pos_idx, n_dof, name);

		// Computing the floating-base Jacobian for a given contact
		wkin_->getFloatingBaseJacobian(base_jac,
									   wkin_->getFrameJacobian(name));

		// Stacking the transpose of the branch Jacobian matrix
		A.block<6,3>(0, 6*idx) = base_jac.topRows<3>().transpose();
		++idx;
	}

	// Mapping the base wrench to contact forces
	idx = 0;
	Eigen::VectorXd lambda = math::pseudoInverse(A) * base_wrench.toVector();
	for (ElementList::const_iterator it = contacts.begin();
			it != contacts.end(); ++it) {
		std::string name = *it;
		Eigen::Vector6d force = lambda.segment<6>(6*idx);
		contact_forces[name] =
				dwl::Force(force.segment<3>(rbd::LX_V),
						   force.segment<3>(rbd::AX_V));
		++idx;
	}
}


const Eigen::MatrixXd&
WholeBodyDynamics::computeJointSpaceInertiaMatrix(const dwl::SE3& base_pos,
												  const Eigen::VectorXd& joint_pos)
{
	// Converting base and joint states to generalized joint states
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);

	// Running the Composite-Rigid-Body Algorithm (CRBA)
	se3::crba(fbs_->getModel(), fbs_->getData(), q);

	return fbs_->getData().M;
}


const Eigen::Matrix6d&
WholeBodyDynamics::computeCentroidalInertiaMatrix(const dwl::SE3& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  const dwl::Motion& base_vel,
												  const Eigen::VectorXd& joint_vel)
{
	// Converting base and joint states to generalized joint states
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	const Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);

	// Running the Centroidal Composite-Rigid-Body Algorithm (CRBA)
	se3::ccrba(fbs_->getModel(), fbs_->getData(), q, qd);

	const se3::Inertia &Icom = fbs_->getData().Ig;
	com_inertia_mat_ = Icom.matrix();

	return com_inertia_mat_;
}


const Eigen::Matrix6x&
WholeBodyDynamics::computeCentroidalMomentumMatrix(const dwl::SE3& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   const dwl::Motion& base_vel,
												   const Eigen::VectorXd& joint_vel)
{
	// Converting base and joint states to generalized joint states
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	const Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);

	// Running the Centroidal Composite-Rigid-Body Algorithm (CRBA)
	se3::ccrba(fbs_->getModel(), fbs_->getData(), q, qd);
	return fbs_->getData().Ag;
}


const dwl::Force&
WholeBodyDynamics::computeGravitoWrench(const dwl::SE3& base_pos,
		   	   	   	   	   	   	   	    const Eigen::VectorXd& joint_pos)
{
	// Computing the CoM position
	Eigen::Vector3d c = wkin_->computeCoM(base_pos, joint_pos);

	// Getting the gravito wrench
	getGravitoWrench(c);

	return grav_wrench_;
}


const dwl::Force&
WholeBodyDynamics::getGravitoWrench(const Eigen::Vector3d& com_pos)
{
	// Computing the weight vector
	const Eigen::Vector3d mg =
			-fbs_->getTotalMass() * fbs_->getGravityVector();

	// Mapping the gravitational wrench in the world frame
	grav_wrench_.setLinear(mg);
	grav_wrench_.setAngular(com_pos.cross(mg));

	return grav_wrench_;
}


void WholeBodyDynamics::estimateContactForces(dwl::ForceMap& contact_forces,
											  const dwl::SE3& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  const dwl::Motion& base_vel,
											  const Eigen::VectorXd& joint_vel,
											  const dwl::Motion& base_acc,
											  const Eigen::VectorXd& joint_acc,
											  const Eigen::VectorXd& joint_forces,
											  const ElementList& contacts)
{
	// Computing the estimated joint forces assuming that there aren't
	// contact forces
	dwl::Force base_wrench;
	Eigen::VectorXd estimated_joint_forces =
			Eigen::VectorXd::Zero(fbs_->getTangentDim());
	computeInverseDynamics(base_wrench, estimated_joint_forces,
						   base_pos, joint_pos,
						   base_vel, joint_vel,
						   base_acc, joint_acc);

	// Computing the joint force error
	Eigen::VectorXd joint_force_error = estimated_joint_forces - joint_forces;

	// Computing the contact forces
	wkin_->updateJacobians(base_pos, joint_pos);
	for (ElementList::const_iterator it = contacts.begin();
			it != contacts.end(); ++it)
	{
		std::string name = *it;

		// Getting the Jacobian of this frame
		Eigen::Matrix6x fixed_jac;
		const Eigen::Matrix6x &frame_jac = wkin_->getFrameJacobian(name);
		wkin_->getFixedBaseJacobian(fixed_jac, frame_jac);

		// Computing the contact force given joint torque error
		unsigned int pos_idx, n_dof;
		fbs_->getBranch(pos_idx, n_dof, name);

		if (n_dof <= 3) {
			Eigen::Matrix3d Jt =
					fixed_jac.block(0, pos_idx, 3, n_dof).transpose();
			Eigen::Vector3d force =
					math::pseudoInverse(Jt) *
					fbs_->getBranchState(joint_force_error, name);
			contact_forces[name] = dwl::Force(force, Eigen::Vector3d::Zero());
		} else {
			Eigen::Matrix6d Jt =
					fixed_jac.block(0, pos_idx, 6, n_dof).transpose();
			Eigen::Vector6d force =
					math::pseudoInverse(Jt) *
					fbs_->getBranchState(joint_force_error, name);
			contact_forces[name] =
					dwl::Force(force.segment<3>(rbd::LX_V),
							   force.segment<3>(rbd::AX_V));
		}
	}
}


void WholeBodyDynamics::estimateGroundReactionForces(dwl::ForceMap& grfs,
													 const Eigen::Vector3d& cop_pos,
													 const dwl::SE3Map& contact_pos,
													 const ElementList& ground_contacts)
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
	for (ElementList::const_iterator it = ground_contacts.begin();
			it != ground_contacts.end(); ++it) {
		std::string name = *it;

		// Getting the contact position
		dwl::SE3Map::const_iterator pos_it = contact_pos.find(name);
		Eigen::VectorXd position;
		if (pos_it != contact_pos.end())
			position = pos_it->second.getTranslation();
		else {
			printf(YELLOW "Warning: there is missing the contact position of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		// Filling the contact position matrix
		contact_mat.col(idx) = position;

		++idx;
	}

	// Computing the normal contact forces
	double weight = fbs_->getTotalMass() * fbs_->getGravityAcceleration();
	Eigen::VectorXd f_normal = math::pseudoInverse(contact_mat) * cop_pos * weight;

	// Filling the contact forces vector
	idx = 0;
	for (ElementList::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); ++contact_iter) {
		std::string name = *contact_iter;

		grfs[name] = dwl::Force(Eigen::Vector3d(0., 0., f_normal[idx]),
								Eigen::Vector3d::Zero());
		++idx;
	}
}


void WholeBodyDynamics::estimateActiveContactsAndForces(ElementList& active_contacts,
														dwl::ForceMap& contact_forces,
														const dwl::SE3& base_pos,
														const Eigen::VectorXd& joint_pos,
														const dwl::Motion& base_vel,
														const Eigen::VectorXd& joint_vel,
														const dwl::Motion& base_acc,
														const Eigen::VectorXd& joint_acc,
														const Eigen::VectorXd& joint_forces,
														const ElementList& contacts,
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


void WholeBodyDynamics::estimateActiveContacts(ElementList& active_contacts,
											   const dwl::SE3& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const dwl::Motion& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const dwl::Motion& base_acc,
											   const Eigen::VectorXd& joint_acc,
											   const Eigen::VectorXd& joint_forces,
											   const ElementList& contacts,
											   double force_threshold)
{
	// Computing the contact forces in predefined set of end-effector
	dwl::ForceMap contact_forces;
	estimateActiveContactsAndForces(active_contacts, contact_forces,
									base_pos, joint_pos,
									base_vel, joint_vel,
									base_acc, joint_acc,
									joint_forces, contacts,
									force_threshold);
}


void WholeBodyDynamics::getActiveContacts(ElementList& active_contacts,
										  const dwl::ForceMap& contact_forces,
										  double force_threshold)
{
	// Detecting active end-effector by using a force threshold
	active_contacts.clear();
	for (dwl::ForceMap::const_iterator it = contact_forces.begin();
			it != contact_forces.end(); ++it) {
		std::string name = it->first;
		const dwl::Force &wrench = it->second;

		if (wrench.toVector().norm() > force_threshold)
			active_contacts.push_back(name);
	}
}


void WholeBodyDynamics::convertAppliedExternalForces(se3::container::aligned_vector<se3::Force>& body_force,
													 const dwl::ForceMap& frame_force)
{
	// Resetting the values of the external force vector
	for (se3::container::aligned_vector<se3::Force>::iterator it = body_force.begin();
			it != body_force.end(); ++it) {
		it->setZero();
	}

	// Mapping the frame force to each parent body
	for (dwl::ForceMap::const_iterator it = frame_force.begin();
			it != frame_force.end(); ++it) {
		// Getting the frame name and force
		const std::string &name = it->first;
		const dwl::Force &force = it->second;

		// Getting the frame information
		unsigned int id = fbs_->getModel().getFrameId(name);
		const se3::Frame &f = fbs_->getModel().frames[id];

		// Computing the transformation between frame to local coordinates of
		// the frame
		const se3::SE3 &f_X_w =
			se3::SE3(fbs_->getData().oMi[f.parent].rotation().transpose(),
					 f.placement.translation());

		// Mapping the body force into the local frame
		body_force[f.parent] = f_X_w.act(force.data);
	}
}


void WholeBodyDynamics::computeCenterOfPressure(Eigen::Vector3d& cop_pos,
												const dwl::ForceMap& contact_for,
												const dwl::SE3Map& contact_pos)
{
	// TODO: Compute the CoM position for case when the normal surface is different to z
	// Initializing the variables
	cop_pos.setZero();
	double sum = 0.;

	// Getting the names of the feet
	ElementList ground_contacts = fbs_->getEndEffectorList(model::FOOT);

	// Sanity check: checking if there are contact information and the size
	if ((contact_for.size() == 0) || (contact_pos.size() == 0) ||
			contact_for.size() != contact_pos.size()) {
		printf(YELLOW "Warning: could not compute the CoP because there is"
				" missing information\n" COLOR_RESET);
		return;
	}

	// Computing the Center of Pressure (CoP) position
	for (ElementList::const_iterator it = ground_contacts.begin();
			it != ground_contacts.end(); ++it) {
		std::string name = *it;

		// Getting the ground reaction forces
		dwl::Force force;
		dwl::ForceMap::const_iterator for_it = contact_for.find(name);
		if (for_it != contact_for.end())
			force = for_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact force of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		// Getting the contact position
		dwl::SE3Map::const_iterator pos_it = contact_pos.find(name);
		dwl::SE3 position;
		if (pos_it != contact_pos.end())
			position = pos_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact position of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		// Accumulate the cop position as a weighted sum, where the weight is
		// the z component of the force at each contact point
		double f_normal = force.toVector()(rbd::LZ_V);
		cop_pos += f_normal * position.getTranslation();
		sum += f_normal;
	}
	cop_pos /= sum;
}


void WholeBodyDynamics::computeZeroMomentPoint(Eigen::Vector3d& zmp_pos,
											   const Eigen::Vector3d& com_pos,
											   const Eigen::Vector3d& com_acc,
											   const double& height)
{
	// Check that the pendulum height is positive
	assert(height >= 0.);

	double omega = sqrt(fbs_->getGravityAcceleration() / height);
	zmp_pos = com_pos -	com_acc / omega;
	zmp_pos(rbd::Z) = com_pos(rbd::Z) - height;
}


void WholeBodyDynamics::computeInstantaneousCapturePoint(Eigen::Vector3d& icp_pos,
														 const Eigen::Vector3d& com_pos,
		                                                 const Eigen::Vector3d& com_vel,
		                                                 const double& height)
{
	// Check that the pendulum height is positive
	assert(height >= 0.);

	double omega = sqrt(fbs_->getGravityAcceleration() / height);
	icp_pos = com_pos + com_vel / omega;
	icp_pos(rbd::Z) = com_pos(rbd::Z) - height;
}


void WholeBodyDynamics::computeCentroidalMomentPivot(Eigen::Vector3d& cmp_pos,
													 const Eigen::Vector3d& com_pos,
													 const double& height,
													 const dwl::ForceMap& contact_for)
{
	// Check that the pendulum height is positive
	assert(height >= 0.);

	// Getting the names of the feet
	ElementList ground_contacts = fbs_->getEndEffectorList(model::FOOT);

	// The Centroidal Momentum Pivot (CMP) is computed given the GRFs
	double grf_x = 0., grf_y = 0., grf_z = 0.;
	for (ElementList::const_iterator it = ground_contacts.begin();
			it != ground_contacts.end(); ++it) {
		std::string name = *it;

		// Getting the ground reaction forces
		dwl::Force force;
		dwl::ForceMap::const_iterator for_it = contact_for.find(name);
		if (for_it != contact_for.end())
			force = for_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact force of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}

		grf_x += force.toVector()(rbd::LX_V);
		grf_y += force.toVector()(rbd::LY_V);
		grf_z += force.toVector()(rbd::LZ_V);
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
										 const dwl::ForceMap& contact_for)
{
	// Getting the names of the feet
	ElementList ground_contacts = fbs_->getEndEffectorList(model::FOOT);

	// The Centroidal Momentum Pivot (CMP) is computed given the GRFs
	double grf_z = 0.;
	for (ElementList::const_iterator contact_iter = ground_contacts.begin();
			contact_iter != ground_contacts.end(); contact_iter++) {
		std::string name = *contact_iter;

		// Getting the ground reaction forces
		dwl::Force force;
		dwl::ForceMap::const_iterator for_it = contact_for.find(name);
		if (for_it != contact_for.end())
			force = for_it->second;
		else {
			printf(YELLOW "Warning: there is missing the contact force of"
					" %s\n" COLOR_RESET, name.c_str());
			return;
		}
		grf_z += force.toVector()(rbd::LZ_V);
	}

	torque(rbd::X) = grf_z * (cop_pos(rbd::Y) - cmp_pos(rbd::Y));
	torque(rbd::Y) = grf_z * (cmp_pos(rbd::X) - cop_pos(rbd::X));
	torque(rbd::Z) = 0.;
}


std::shared_ptr<FloatingBaseSystem> WholeBodyDynamics::getFloatingBaseSystem()
{
	return fbs_;
}


std::shared_ptr<WholeBodyKinematics> WholeBodyDynamics::getWholeBodyKinematics()
{
	return wkin_;
}


} //@namespace model
} //@namespace dwl
