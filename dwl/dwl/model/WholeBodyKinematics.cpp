#include <dwl/model/WholeBodyKinematics.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics(): step_tol_(1.0e-12), max_iter_(10)
{

}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::reset(FloatingBaseSystem& fbs)
{
	// Creating a shared pointer of floating-base system object
	fbs_ = std::make_shared<FloatingBaseSystem>(fbs);

	// Computing the middle value for IK routines
	joint_pos_middle_ = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	for (unsigned int j = 0; j < fbs_->getJointDoF(); ++j) {
		double lower_limit = fbs_->getLowerLimits()[j];
		double upper_limit = fbs_->getUpperLimits()[j];
		joint_pos_middle_(j) = (upper_limit + lower_limit) / 2;
	}
}



void WholeBodyKinematics::setIKSolver(double step_tol,
									  unsigned int max_iter)
{
	step_tol_ = step_tol;
	max_iter_ = max_iter;
}


void WholeBodyKinematics::updateKinematics(dwl::SE3& base_pos,
										   const Eigen::VectorXd& joint_pos)
{
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q);
}


void WholeBodyKinematics::updateKinematics(dwl::SE3& base_pos,
										   const Eigen::VectorXd& joint_pos,
										   const dwl::Motion& base_vel,
										   const Eigen::VectorXd& joint_vel)
{
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	const Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q, qd);
}


void WholeBodyKinematics::updateKinematics(dwl::SE3& base_pos,
										   const Eigen::VectorXd& joint_pos,
										   const dwl::Motion& base_vel,
										   const Eigen::VectorXd& joint_vel,
										   const dwl::Motion& base_acc,
										   const Eigen::VectorXd& joint_acc)
{
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	const Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	const Eigen::VectorXd qdd = fbs_->toTangentState(base_acc, joint_acc);
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q, qd, qdd);
}


void WholeBodyKinematics::updateJacobians(dwl::SE3& base_pos,
										  const Eigen::VectorXd& joint_pos)
{
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	se3::computeJointJacobians(fbs_->getModel(), fbs_->getData(), q);
	se3::framesForwardKinematics(fbs_->getModel(), fbs_->getData());
}


const Eigen::Vector3d&
WholeBodyKinematics::computeCoM(dwl::SE3& base_pos,
								const Eigen::VectorXd& joint_pos)
{
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	return se3::centerOfMass(fbs_->getModel(), fbs_->getData(), q);
}


void WholeBodyKinematics::computeCoMRate(Eigen::Vector3d& com,
										 Eigen::Vector3d& com_d,
										 Eigen::Vector3d& com_dd,
										 dwl::SE3& base_pos,
										 const Eigen::VectorXd& joint_pos,
										 const dwl::Motion& base_vel,
										 const Eigen::VectorXd& joint_vel,
										 const dwl::Motion& base_acc,
										 const Eigen::VectorXd& joint_acc)
{
	const Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	const Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	const Eigen::VectorXd qdd = fbs_->toTangentState(base_acc, joint_acc);

	se3::centerOfMass(fbs_->getModel(), fbs_->getData(), q, qd, qdd);

	com = fbs_->getData().com[0];
	com_d = fbs_->getData().vcom[0];
	com_dd = fbs_->getData().acom[0];
}


dwl::SE3Map
WholeBodyKinematics::computePosition(dwl::SE3& base_pos,
									 const Eigen::VectorXd& joint_pos,
									 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos);
	
	dwl::SE3Map frame_pos;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_pos[name] = getFramePosition(name);
	}

	return frame_pos;
}


const dwl::SE3&
WholeBodyKinematics::getFramePosition(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];
	
	se3_.data = fbs_->getData().oMi[f.parent].act(f.placement);

	return se3_;
}


dwl::MotionMap
WholeBodyKinematics::computeVelocity(dwl::SE3& base_pos,
									 const Eigen::VectorXd& joint_pos,
									 const dwl::Motion& base_vel,
									 const Eigen::VectorXd& joint_vel,
									 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos, base_vel, joint_vel);

	dwl::MotionMap frame_vel;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_vel[name] = getFrameVelocity(name);
	}

	return frame_vel;
}


const dwl::Motion&
WholeBodyKinematics::getFrameVelocity(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];

	// Computing the transformation between world to local coordinates of the
	// frame
	const se3::SE3 &f_X_w =
		se3::SE3(fbs_->getData().oMi[f.parent].rotation().transpose(),
				 f.placement.translation());

	// Converting the frame velocity in local coordinate to world coordinate
	// Note that this is equivalent to convert the body spatial velocity into
	// the local coordinates of the frame (i.e. f_v_l = f.placement.actInv(data.v))
	// and the then apply the rotation between the world and frame (i.e. w_R_f)
	// but more efficient because it's needed only 1 motion transform
	motion_.data = f_X_w.actInv(fbs_->getData().v[f.parent]);
	return motion_;
}


dwl::MotionMap
WholeBodyKinematics::computeAcceleration(dwl::SE3& base_pos,
										 const Eigen::VectorXd& joint_pos,
										 const dwl::Motion& base_vel,
										 const Eigen::VectorXd& joint_vel,
										 const dwl::Motion& base_acc,
										 const Eigen::VectorXd& joint_acc,
										 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos,
					 base_vel, joint_vel,
					 base_acc, joint_acc);

	dwl::MotionMap frame_acc;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_acc[name] = getFrameAcceleration(name);
	}

	return frame_acc;
}


const dwl::Motion&
WholeBodyKinematics::getFrameAcceleration(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];

	// Computing the transformation between world to local coordinates of the
	// frame
	const se3::SE3 &f_X_w =
		se3::SE3(fbs_->getData().oMi[f.parent].rotation().transpose(),
				 f.placement.translation());

	// Converting the frame velocity in local coordinate to world coordinate
	const se3::Motion &vel = f_X_w.actInv(fbs_->getData().v[f.parent]);

	// Converting the frame acceleration in local coordinate to world coordinate
	// Note that this is equivalent to convert the body spatial acceleration into
	// the local coordinates of the frame (i.e. f_v_l = f.placement.actInv(data.a))
	// and the then apply the rotation between the world and frame (i.e. w_R_f)
	// but more efficient because it's needed only 1 motion transform
	motion_.data = f_X_w.actInv(fbs_->getData().a[f.parent]);
	motion_.data.linear() += vel.angular().cross(vel.linear());

	return motion_;
}


dwl::MotionMap
WholeBodyKinematics::computeJdQd(dwl::SE3& base_pos,
								 const Eigen::VectorXd& joint_pos,
								 const dwl::Motion& base_vel,
								 const Eigen::VectorXd& joint_vel,
								 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos,
					 base_vel, joint_vel,
					 dwl::Motion(), Eigen::VectorXd::Zero(fbs_->getJointDoF()));


	dwl::MotionMap frame_jdqd;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		// Since we set up the acceleration vector null, then the Jdot * qdot
		// term is equals to the accelerations
		frame_jdqd[name] = getFrameJdQd(name);
	}

	return frame_jdqd;
}


const dwl::Motion&
WholeBodyKinematics::getFrameJdQd(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];

	// Computing the transformation between world to local coordinates of the frame
	const se3::SE3 &f_X_w =
		se3::SE3(fbs_->getData().oMi[f.parent].rotation().transpose(),
				 f.placement.translation());

	// Converting the frame velocity in local coordinate to world coordinate
	const se3::Motion &vel = f_X_w.actInv(fbs_->getData().v[f.parent]);
	const se3::Motion &acc =
			f_X_w.actInv(fbs_->getData().a[f.parent]) +
			se3::Motion(vel.angular().cross(vel.linear()),
						Eigen::Vector3d::Zero());

	motion_.data.linear() = acc.angular();
	motion_.data.angular() = acc.linear() + vel.angular().cross(vel.linear());

	return motion_;
}


bool WholeBodyKinematics::computeJointPosition(Eigen::VectorXd& joint_pos,
											   const dwl::SE3Map& frame_pos)
{
	return computeJointPosition(joint_pos, frame_pos, joint_pos_middle_);
}


bool WholeBodyKinematics::computeJointPosition(Eigen::VectorXd& joint_pos,
											   const dwl::SE3Map& frame_pos,
											   const Eigen::VectorXd& joint_pos0)
{
	// Indicates if we manage to solve the IK problem
	bool success = false;

	// Setting up the warm-start joint point
	dwl::SE3 se3_origin;
	dwl::Motion zero_motion;
	Eigen::VectorXd q = fbs_->toConfigurationState(se3_origin, joint_pos0);

	// Getting the offset in the joint vector in case of floating-base systems
	unsigned int n_ff_q = 0, n_ff_v = 0;
	if (fbs_->isFloatingBase()) {
		n_ff_q = 7;
		n_ff_v = 6;
	}

	// Created an internal copy of the frame positions
	dwl::SE3Map frame_list = frame_pos;

	// Iterating until a satisfied the desired tolerance or reach the maximum
	// number of iterations
	for (unsigned int k = 0; k < max_iter_; ++k) {
		// Updating the Jacobians and frame kinematics
		se3::computeJointJacobians(fbs_->getModel(), fbs_->getData(), q);
		se3::framesForwardKinematics(fbs_->getModel(), fbs_->getData());

		for (auto it = frame_list.cbegin(), next_it = frame_list.cbegin();
				it != frame_list.cend(); it = next_it) {
			next_it = it; ++next_it;
			// Getting the frame information
			std::string name = it->first;
			const dwl::SE3& X_des = it->second;
			unsigned int id = fbs_->getModel().getFrameId(name);

			// Compute the frame to base transformation. Note that the reference
			// systems world and frame are aligned
			const se3::SE3 &b_X_f = fbs_->getData().oMf[id];

			// Computing the frame Jacobian (of actuation part) in the base frame
			Eigen::Matrix6x full_jac(6,fbs_->getTangentDim());
			Eigen::Matrix6x fixed_jac(6,fbs_->getTangentDim());
			full_jac.setZero();
			se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, full_jac);
			getFixedBaseJacobian(fixed_jac, full_jac);
			fixed_jac = b_X_f.toDualActionMatrix() * fixed_jac;

			// Compute the Gauss-Newton direction, the residual error is computed
			// according the current-branch DoF
			unsigned int pos_idx, n_dof;
			fbs_->getBranch(pos_idx, n_dof, name);
			Eigen::VectorXd qdot;
			if (n_dof <= 3) {// only position error
				Eigen::Vector3d e = b_X_f.translation() - X_des.getTranslation();
				qdot = fbs_->toTangentState(zero_motion,
											-math::pseudoInverse(fixed_jac.topRows<3>()) * e);
			} else {// SE3 error
				Eigen::Vector6d e = se3::log6(X_des.data.inverse() * b_X_f);
				qdot = fbs_->toTangentState(zero_motion,
											-math::pseudoInverse(fixed_jac * e));
			}

			// Integrate the exp integration in SE3
			q = se3::integrate(fbs_->getModel(), q, qdot);

			if (qdot.segment(n_ff_v + pos_idx, n_dof).norm() < step_tol_) {
				frame_list.erase(it);

				// Copying the values of this branch
				joint_pos.segment(pos_idx, n_dof) =
						q.segment(n_ff_q + pos_idx, n_dof);

				// Return solution when there are all iteration convergence
				if (frame_list.size() == 0) {
					success = true;
					return success;
				}
			}
		}
	}

	// Returning the most updated joint value even if the IK didn't convergence
	for (dwl::SE3Map::const_iterator it = frame_list.begin();
			it != frame_list.end(); ++it) {
		// Computing the branch properties
		unsigned int pos_idx, n_dof;
		fbs_->getBranch(pos_idx, n_dof, it->first);
		joint_pos.segment(pos_idx, n_dof) =
				q.segment(n_ff_q + pos_idx, n_dof);
	}

	return success;
}


void WholeBodyKinematics::computeJointVelocity(Eigen::VectorXd& joint_vel,
											   const Eigen::VectorXd& joint_pos,
											   const dwl::MotionMap& frame_vel)
{
	// Getting the configuration vector and updating the frame Jacobians and
	// kinematics
	dwl::SE3 se3_origin;
	dwl::Motion zero_motion;
	updateKinematics(se3_origin, joint_pos,
					 zero_motion, Eigen::VectorXd::Zero(fbs_->getJointDoF()),
					 zero_motion, Eigen::VectorXd::Zero(fbs_->getJointDoF()));

	// Getting the joint velocities
	getJointVelocity(joint_vel, frame_vel);
}


void WholeBodyKinematics::getJointVelocity(Eigen::VectorXd& joint_vel,
										   const dwl::MotionMap& frame_vel)
{
	// Checks vector dimension
	assert(joint_vel.size() == fbs_->getJointDoF());

	for (dwl::MotionMap::const_iterator it = frame_vel.begin();
		it != frame_vel.end(); ++it) {
		// Getting the frame information
		std::string name = it->first;
		const dwl::Motion& v_frame = it->second;
		unsigned int id = fbs_->getModel().getFrameId(name);

		// Computing the transformation between the local to world coordinates
		// of the frame
		const se3::SE3 &w_X_f = se3::SE3(fbs_->getData().oMf[id].rotation(),
										 Eigen::Vector3d::Zero());

		// Computing the frame Jacobian (actuated part) in the base frame
		Eigen::Matrix6x full_jac(6, fbs_->getTangentDim());
		Eigen::Matrix6x fixed_jac(6, fbs_->getTangentDim());
		full_jac.setZero();
		se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, full_jac);
		getFixedBaseJacobian(fixed_jac, full_jac);
		fixed_jac = w_X_f.toActionMatrix() * fixed_jac;

		// Getting the branch Jacobian
		unsigned int pos_idx, n_dof;
		fbs_->getBranch(pos_idx, n_dof, name);
		Eigen::Matrix6x branch_jac = fixed_jac.block(0, pos_idx, 6, n_dof);

		// Computing the branch joint velocity
		Eigen::VectorXd branch_joint_vel;
		if (n_dof <= 3) {// only cartesian velocity
			branch_joint_vel =
				math::pseudoInverse(branch_jac.topRows<3>()) * v_frame.getLinear();
		} else { // 6d velocity
			branch_joint_vel =
				math::pseudoInverse(branch_jac) * v_frame.toVector();
		}

		// Setting up the branch joint velocity
		fbs_->setBranchState(joint_vel, branch_joint_vel, name);
	}
}


void WholeBodyKinematics::computeJointAcceleration(Eigen::VectorXd& joint_acc,
												   const Eigen::VectorXd& joint_pos,
												   const Eigen::VectorXd& joint_vel,
												   const dwl::MotionMap& frame_acc)
{
	// Update the kinematics
	dwl::SE3 se3_origin;
	dwl::Motion zero_motion;
	updateKinematics(se3_origin, joint_pos,
					 zero_motion, joint_vel,
					 zero_motion, Eigen::VectorXd::Zero(fbs_->getJointDoF()));

	// Updating the frame Jacobians and kinematics
	updateJacobians(se3_origin, joint_pos);

	// Getting the joint acceleration
	getJointAcceleration(joint_acc, frame_acc);
}


void WholeBodyKinematics::getJointAcceleration(Eigen::VectorXd& joint_acc,
											   const dwl::MotionMap& frame_acc)
{
	// Checks vector dimension
	assert(joint_acc.size() == fbs_->getJointDoF());

	for (dwl::MotionMap::const_iterator it = frame_acc.begin();
		it != frame_acc.end(); ++it) {
		// Getting the frame information
		std::string name = it->first;
		const dwl::Motion& a_frame = it->second;
		unsigned int id = fbs_->getModel().getFrameId(name);

		// Computing the transformation between the local to world coordinates
		// of the frame
		const se3::SE3 &w_X_f = se3::SE3(fbs_->getData().oMf[id].rotation(),
										 Eigen::Vector3d::Zero());

		// Computing the frame Jacobian (actuated part) in the base frame
		Eigen::Matrix6x full_jac(6, fbs_->getTangentDim());
		Eigen::Matrix6x fixed_jac(6, fbs_->getTangentDim());
		full_jac.setZero();
		se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, full_jac);
		getFixedBaseJacobian(fixed_jac, full_jac);
		fixed_jac = w_X_f.toActionMatrix() * fixed_jac;

		// Getting the branch Jacobian
		unsigned int pos_idx, n_dof;
		fbs_->getBranch(pos_idx, n_dof, name);
		Eigen::Matrix6x branch_jac = fixed_jac.block(0, pos_idx, 6, n_dof);

		// Computing the Jd*qd term expressed in the base frame
		// Since we set up the acceleration vector null, then the Jdot * qdot
		// term is equals to the accelerations
		const dwl::Motion& jd_qd = getFrameJdQd(name);

		// Computing the branch joint velocity
		Eigen::VectorXd branch_joint_acc;
		if (n_dof <= 3) {// only cartesian velocity
			branch_joint_acc =
				math::pseudoInverse(branch_jac.topRows<3>()) *
					(a_frame.getLinear() - jd_qd.getLinear());
		} else { // 6d velocity
			branch_joint_acc =
				math::pseudoInverse(branch_jac) *
					(a_frame.toVector() - jd_qd.toVector());
		}

		// Setting up the branch joint acceleration
		fbs_->setBranchState(joint_acc, branch_joint_acc, name);
	}
}


void WholeBodyKinematics::computeConstrainedJointAcceleration(Eigen::VectorXd& joint_acc,
															  dwl::SE3& base_pos,
															  const Eigen::VectorXd& joint_pos,
															  const dwl::Motion& base_vel,
															  const Eigen::VectorXd& joint_vel,
															  const dwl::Motion& base_acc,
															  const ElementList& contacts)
{
	// Updating the frame Jacobians and kinematics
	updateKinematics(base_pos, joint_pos,
					 base_vel, joint_vel);
	updateJacobians(base_pos, joint_pos);

	// Getting the constrained joint acceleration
	getConstrainedJointAcceleration(joint_acc, base_vel, base_acc, contacts);
}


void WholeBodyKinematics::getConstrainedJointAcceleration(Eigen::VectorXd& joint_acc,
														  const dwl::Motion& base_vel,
														  const dwl::Motion& base_acc,
														  const ElementList& contacts)
{
	// Checks vector dimension
	assert(joint_acc.size() == fbs_->getJointDoF());

	// At the first step, we compute the angular and linear floating-base
	// velocity and acceleration
	const Eigen::Vector3d x_d = base_vel.getLinear();
	const Eigen::Vector3d omega = base_vel.getAngular();
	const Eigen::Vector3d x_dd = base_acc.getLinear();
	const Eigen::Vector3d omega_d = base_acc.getAngular();

	// Computing the consistent joint accelerations given a desired base
	// motion and contact definition. We assume that contacts are static,
	// which it allows us to computed a consistent joint accelerations.
	dwl::SE3 r;
	dwl::Motion rd, rdd;
	Eigen::Matrix6x fixed_jac;
	for (ElementList::const_iterator it = contacts.begin();
			it != contacts.end(); ++it) {
		std::string name = *it;

		// Getting the fixed Jacobian and Jd*qd term
		getFixedBaseJacobian(fixed_jac, getFrameJacobian(name));
		const dwl::Motion Jdqd = getFrameJdQd(name);

		// Computing the desired contact acceleration w.r.t. the base frame
		r = getFramePosition(name);
		rd.setLinear(-x_d - omega.cross(r.getTranslation()));
		rd.setAngular(-omega);
		rdd.setLinear(-x_dd
					  -omega_d.cross(r.getTranslation())
					  -omega.cross(r.getTranslation())
					  -2 * omega.cross(rd.getLinear()));
		rdd.setAngular(-omega_d);

		// Get the branch properties
		unsigned int pos_idx, n_dof;
		fbs_->getBranch(pos_idx, n_dof, name);

		// Computing the join acceleration from r_dd = J*q_dd + J_d*q_d
		// since we are doing computation in the base frame
		Eigen::VectorXd q_dd;
		if (n_dof <= 3) {// only position error
			Eigen::Matrix3d J_branch = fixed_jac.block<3,3>(0, pos_idx);
			q_dd =
				math::pseudoInverse(J_branch) * (rdd.getLinear() - Jdqd.getLinear());
		} else {
			Eigen::Matrix6d J_branch = fixed_jac.block<6,6>(0, pos_idx);
			q_dd =
				math::pseudoInverse(J_branch) * (rdd.toVector() - Jdqd.toVector());
		}
		// Setting up the branch joint acceleration
		fbs_->setBranchState(joint_acc, q_dd, name);
	}
}


Eigen::Matrix6xMap
WholeBodyKinematics::computeJacobian(dwl::SE3& base_pos,
									 const Eigen::VectorXd& joint_pos,
									 const ElementList& frames)
{
	// Creates the Jacobian variables
	Eigen::Matrix6xMap frame_jac;
	Eigen::Matrix6x ordered_jac;
	ordered_jac.resize(6, fbs_->getTangentDim());

	// Updating the frame Jacobians and kinematics
	updateJacobians(base_pos, joint_pos);

	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_jac[name] = getFrameJacobian(name);
	}

	return frame_jac;
}


Eigen::Matrix6x
WholeBodyKinematics::getFrameJacobian(const std::string& name)
{
	// Getting frame and its joint properties
	unsigned int id = fbs_->getModel().getFrameId(name);
    const se3::Frame &frame = fbs_->getModel().frames[id];
    const se3::Model::JointIndex &joint_id = frame.parent;

	// Getting the body Jacobian
	Eigen::Matrix6x J(6, fbs_->getTangentDim());
	J.setZero();
	se3::getFrameJacobian<se3::WORLD>(fbs_->getModel(), fbs_->getData(), id, J);

	// Transforming the body Jacobian into the frame Jacobian in local
	// coordinates is equivalent to f^J = T^-1*i^J where T = [R , [p]x*R; 0 R].
	// Furthermore expressing the frame Jacobian in world frame requires the
	// following transformation o^J = [R, 0; 0, R]*f^J. And it can be shown
	// easily that o^J = M*i^J where M = [I, -[p]x; 0, I]. Note that M
	// can be compute using coordinate transform for SE3(I, -p).
	const se3::SE3 &w_X_f = se3::SE3(Eigen::Matrix3d::Identity(),
									  -fbs_->getData().oMf[id].translation());

	// Sparse computation of the M*i^J. Given a frame (or its joint), we find
	// its joint index and move recursively using the sparsity pattern defined
	// by parents_fromRow()
	int j_idx = se3::nv(fbs_->getModel().joints[joint_id]) +
			se3::idx_v(fbs_->getModel().joints[joint_id]) - 1;
    for (int j = j_idx; j >= 0;
    		j = fbs_->getData().parents_fromRow[(size_t) j]) {
    	J.col(j) = w_X_f.act(se3::Motion(J.col(j))).toVector();
    }

    return J;
}


void WholeBodyKinematics::getFloatingBaseJacobian(Eigen::Matrix6d& jacobian,
												  const Eigen::Matrix6x& full_jacobian)
{
	if (fbs_->isFloatingBase())
		jacobian = full_jacobian.leftCols<6>();
	else
		jacobian.setZero();
}


void WholeBodyKinematics::getFixedBaseJacobian(Eigen::Matrix6x& jacobian,
											   const Eigen::Matrix6x& full_jacobian)
{
	jacobian = full_jacobian.rightCols(fbs_->getJointDoF());
}

} //@namespace model
} //@namespace dwl
