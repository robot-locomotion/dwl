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


void WholeBodyKinematics::updateKinematics(const Eigen::Vector7d& base_pos,
										   const Eigen::VectorXd& joint_pos)
{
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q);
}


void WholeBodyKinematics::updateKinematics(const Eigen::Vector7d& base_pos,
										   const Eigen::VectorXd& joint_pos,
										   const Eigen::Vector6d& base_vel,
										   const Eigen::VectorXd& joint_vel)
{
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q, qd);
}


void WholeBodyKinematics::updateKinematics(const Eigen::Vector7d& base_pos,
										   const Eigen::VectorXd& joint_pos,
										   const Eigen::Vector6d& base_vel,
										   const Eigen::VectorXd& joint_vel,
										   const Eigen::Vector6d& base_acc,
										   const Eigen::VectorXd& joint_acc)
{
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	Eigen::VectorXd qdd = fbs_->toTangentState(base_acc, joint_acc);
	se3::forwardKinematics(fbs_->getModel(), fbs_->getData(), q, qd, qdd);
}


void WholeBodyKinematics::updateJacobians(const Eigen::Vector7d& base_pos,
										  const Eigen::VectorXd& joint_pos)
{
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	computeJacobians(fbs_->getModel(), fbs_->getData(), q);
	framesForwardKinematics(fbs_->getModel(), fbs_->getData());
}


const Eigen::Vector3d&
WholeBodyKinematics::computeCoM(const Eigen::Vector7d& base_pos,
								const Eigen::VectorXd& joint_pos)
{
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	return se3::centerOfMass(fbs_->getModel(), fbs_->getData(), q);
}


void WholeBodyKinematics::computeCoMRate(Eigen::Vector3d& com,
										 Eigen::Vector3d& com_d,
										 const Eigen::Vector7d& base_pos,
										 const Eigen::VectorXd& joint_pos,
										 const Eigen::Vector6d& base_vel,
										 const Eigen::VectorXd& joint_vel)
{
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos, joint_pos);
	Eigen::VectorXd qd = fbs_->toTangentState(base_vel, joint_vel);
	se3::centerOfMass(fbs_->getModel(), fbs_->getData(), q, qd);

	com = fbs_->getData().com[0];
	com_d = fbs_->getData().vcom[0];
}


Eigen::Vector7dMap
WholeBodyKinematics::computePosition(const Eigen::Vector7d& base_pos,
									 const Eigen::VectorXd& joint_pos,
									 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos);
	
	Eigen::Vector7dMap frame_pos;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_pos[name] = getFramePosition(name);
	}

	return frame_pos;
}


const Eigen::Vector7d&
WholeBodyKinematics::getFramePosition(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];
	
	se3::SE3 pos = fbs_->getData().oMi[f.parent].act(f.placement);
	Eigen::Quaterniond q = math::getQuaternion(pos.rotation());
	config_vec_ << q.x(), q.y(), q.z(), q.w(), pos.translation();

	return config_vec_;
}


Eigen::Vector6dMap
WholeBodyKinematics::computeVelocity(const Eigen::Vector7d& base_pos,
									 const Eigen::VectorXd& joint_pos,
									 const Eigen::Vector6d& base_vel,
									 const Eigen::VectorXd& joint_vel,
									 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos, base_vel, joint_vel);

	Eigen::Vector6dMap frame_vel;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_vel[name] = getFrameVelocity(name);
	}

	return frame_vel;
}


const Eigen::Vector6d&
WholeBodyKinematics::getFrameVelocity(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];
	
	// Computing the transformation between world to local coordinates of the frame
	const se3::SE3 &f_X_w = 
		se3::SE3(fbs_->getData().oMi[f.parent].rotation().transpose(),
				 f.placement.translation());

	// Converting the frame velocity in local coordinate to world coordinate
	// Note that this is equivalent to convert the body spatial velocity into
	// the local coordinates of the frame (i.e. f_v_l = f.placement.actInv(data.v))
	// and the then apply the rotation between the world and frame (i.e. w_R_f) but
	// more efficient because it's needed only 1 motion transform
	const se3::Motion &vel = f_X_w.actInv(fbs_->getData().v[f.parent]);
	tangent_vec_ << vel.angular(), vel.linear();

	return tangent_vec_;
}


Eigen::Vector6dMap
WholeBodyKinematics::computeAcceleration(const Eigen::Vector7d& base_pos,
										 const Eigen::VectorXd& joint_pos,
										 const Eigen::Vector6d& base_vel,
										 const Eigen::VectorXd& joint_vel,
										 const Eigen::Vector6d& base_acc,
										 const Eigen::VectorXd& joint_acc,
										 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos, base_vel, joint_vel, base_acc, joint_acc);

	Eigen::Vector6dMap frame_acc;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		frame_acc[name] = getFrameAcceleration(name);
	}

	return frame_acc;
}


const Eigen::Vector6d&
WholeBodyKinematics::getFrameAcceleration(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);
	const se3::Frame &f = fbs_->getModel().frames[id];
	
	// Computing the transformation between world to local coordinates of the frame
	const se3::SE3 &f_X_w = 
		se3::SE3(fbs_->getData().oMi[f.parent].rotation().transpose(),
				 f.placement.translation());

	// Converting the frame velocity in local coordinate to world coordinate
	const se3::Motion &vel = f_X_w.actInv(fbs_->getData().v[f.parent]);

	// Converting the frame acceleration in local coordinate to world coordinate
	// Note that this is equivalent to convert the body spatial acceleration into
	// the local coordinates of the frame (i.e. f_v_l = f.placement.actInv(data.a))
	// and the then apply the rotation between the world and frame (i.e. w_R_f) but
	// more efficient because it's needed only 1 motion transform
	const se3::Motion &acc_dash =
		se3::Motion(vel.angular().cross(vel.linear()), Eigen::Vector3d::Zero());
	const se3::Motion &acc = f_X_w.actInv(fbs_->getData().a[f.parent]) + acc_dash;
	tangent_vec_ << acc.angular(), acc.linear();

	return tangent_vec_;
}


Eigen::Vector6dMap
WholeBodyKinematics::computeJdQd(const Eigen::Vector7d& base_pos,
								 const Eigen::VectorXd& joint_pos,
								 const Eigen::Vector6d& base_vel,
								 const Eigen::VectorXd& joint_vel,
								 const ElementList& frames)
{
	// Update the kinematics
	updateKinematics(base_pos, joint_pos,
					 base_vel, joint_vel,
					 Eigen::Vector6d::Zero(), Eigen::VectorXd::Zero(fbs_->getJointDoF()));


	Eigen::Vector6dMap frame_jdqd;
	for (ElementList::const_iterator it = frames.begin();
		it != frames.end(); ++it) {
		std::string name = *it;
		// Since we set up the acceleration vector null, then the Jdot * qdot term
		// is equals to the accelerations
		frame_jdqd[name] = getFrameAcceleration(name); //TODO create own method
	}

	return frame_jdqd;
}



bool WholeBodyKinematics::computeJointPosition(Eigen::VectorXd& joint_pos,
											   const Eigen::Vector7dMap& frame_pos)
{
	return computeJointPosition(joint_pos, frame_pos, joint_pos_middle_);
}


bool WholeBodyKinematics::computeJointPosition(Eigen::VectorXd& joint_pos,
											   const Eigen::Vector7dMap& frame_pos,
											   const Eigen::VectorXd& joint_pos0)
{
	// Indicates if we manage to solve the IK problem
	bool success = false;

	// Setting up the warm-start joint point
	joint_pos = joint_pos0;
	Eigen::Vector7d base_pos0 = Eigen::Vector7d::Zero();
	base_pos0(rbd::AW_Q) = 1.;
	Eigen::VectorXd q = fbs_->toConfigurationState(base_pos0, joint_pos0);

	// Created an internal copy of the frame positions
	Eigen::Vector7dMap frame_list = frame_pos;

	// Iterating until a satisfied the desired tolerance or reach the maximum
	// number of iterations
	for (unsigned int k = 0; k < max_iter_; ++k) {
		// Updating the Jacobians and frame kinematics
		computeJacobians(fbs_->getModel(), fbs_->getData(), q);
		framesForwardKinematics(fbs_->getModel(), fbs_->getData());
		
		for (Eigen::Vector7dMap::iterator it = frame_list.begin();
			it != frame_list.end(); ++it) {
			// Getting the frame information
			std::string name = it->first;
			Eigen::Vector7d x_des = it->second;
			unsigned int id = fbs_->getModel().getFrameId(name);

			// Compute the frame to base transformation. Note that the reference
			// systems world and frame are aligned
			const se3::SE3 &b_X_f = fbs_->getData().oMf[id];

			// Computing the frame Jacobian (of actuation part) in the base frame
			Eigen::Matrix6x full_jac(6,fbs_->getModel().nv);
			Eigen::Matrix6x fixed_jac(6,fbs_->getModel().nv);
			full_jac.setZero();
			se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, full_jac);
			getFixedBaseJacobian(fixed_jac, full_jac);
			fixed_jac = b_X_f.toDualActionMatrix() * fixed_jac;

			// Compute the Gauss-Newton direction, the residual error is computed
			// according the current-branch DoF
			Eigen::VectorXd qdot;
			unsigned int pos_idx, n_dof;
			fbs_->getBranch(pos_idx, n_dof, name);
			if (n_dof <= 3) {// only position error
				Eigen::Vector3d e = b_X_f.translation() - rbd::linearPart(x_des);

				// Note that the jacobian in Pinocchio is organized as [linear, angular], and
				// the base component isn't considered
				qdot = fbs_->toTangentState(Eigen::Vector6d::Zero(),
											-math::pseudoInverse(fixed_jac.topRows<3>()) * e);
			} else {// SE3 error
				Eigen::Quaterniond quat(x_des(rbd::AW_Q),
										x_des(rbd::AX_Q),
										x_des(rbd::AY_Q),
										x_des(rbd::AZ_Q));
				se3::SE3 M_des = se3::SE3(math::getRotationMatrix(quat),
										  rbd::linearPart(x_des));
				Eigen::Vector6d e = se3::log6(M_des.inverse() * b_X_f);

				// Note that the jacobian in Pinocchio is organized as [linear, angular], and
				// the base component isn't considered
				qdot = fbs_->toTangentState(Eigen::Vector6d::Zero(),
											-math::pseudoInverse(fixed_jac * e));
			}
	
			// Integrate the exp integration in SE3		
			q = se3::integrate(fbs_->getModel(), q, qdot);

			if (qdot.norm() < step_tol_) {
				frame_list.erase(it);

				// Return solution when there are all iteration convergence
				if (frame_list.size() == 0) {
					joint_pos = q.tail(fbs_->getJointDoF());
					success = true;
					return success;
				}
			}
		}
	}

	// Returning the most updated joint value even if the IK didn't convergence
	joint_pos = q.tail(fbs_->getJointDoF());

	return success;
}


void WholeBodyKinematics::computeJointVelocity(Eigen::VectorXd& joint_vel,
											   const Eigen::VectorXd& joint_pos,
											   const Eigen::Vector6dMap& frame_vel)
{
	// Getting the configuration vector and updating the frame Jacobians and kinematics
	Eigen::Vector7d base_pos0 = Eigen::Vector7d::Zero();
	base_pos0(rbd::AW_Q) = 1.;
	updateKinematics(base_pos0, joint_pos,
					 Eigen::Vector6d::Zero(), joint_vel,
					 Eigen::Vector6d::Zero(), Eigen::VectorXd::Zero(fbs_->getJointDoF()));

	// Getting the joint velocities
	getJointVelocity(joint_vel, frame_vel);
}


void WholeBodyKinematics::getJointVelocity(Eigen::VectorXd& joint_vel,
										   const Eigen::Vector6dMap& frame_vel)
{
	// Resizing the vector
	joint_vel = Eigen::VectorXd::Zero(fbs_->getJointDoF());

	// Getting the world to base transform
	unsigned int base_id = fbs_->getModel().getFrameId(fbs_->getFloatingBaseName());
	const se3::SE3 &w_X_b = fbs_->getData().oMf[base_id];

	for (Eigen::Vector6dMap::const_iterator it = frame_vel.begin();
		it != frame_vel.end(); ++it) {
		// Getting the frame information
		std::string name = it->first;
		Eigen::Vector6d v_frame = it->second;
		v_frame << rbd::linearPart(v_frame), rbd::angularPart(v_frame);
		unsigned int id = fbs_->getModel().getFrameId(name);
		
		// Computing the transformation between the local to base coordinates of the frame
		const se3::SE3 &w_X_f = se3::SE3(fbs_->getData().oMf[id].rotation(),
										 Eigen::Vector3d::Zero());
		Eigen::Matrix4d b_H_f =
			w_X_b.toHomogeneousMatrix().inverse() * w_X_f.toHomogeneousMatrix();
		const se3::SE3 &b_X_f = 
			se3::SE3(b_H_f.block<3,3>(0,0), b_H_f.block<3,1>(0,3)); // (R,t)

		// Computing the frame Jacobian (actuated part) in the base frame
		Eigen::Matrix6x full_jac(6, fbs_->getTangentDim());
		Eigen::Matrix6x fixed_jac(6, fbs_->getTangentDim());
		full_jac.setZero();
		se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, full_jac);
		getFixedBaseJacobian(fixed_jac, full_jac);
		fixed_jac = b_X_f.toActionMatrix() * fixed_jac;

		// Computing the branch joint velocity
		Eigen::VectorXd branch_joint_vel =
			fbs_->getBranchState(math::pseudoInverse(fixed_jac) * v_frame, name);

		// Setting up the branch joint velocity
		fbs_->setBranchState(joint_vel, branch_joint_vel, name);
	}
}


void WholeBodyKinematics::computeJointAcceleration(Eigen::VectorXd& joint_acc,
												   const Eigen::VectorXd& joint_pos,
												   const Eigen::VectorXd& joint_vel,
												   const Eigen::Vector6dMap& frame_acc)
{
	// Update the kinematics
	Eigen::Vector7d base_pos0 = Eigen::Vector7d::Zero();
	base_pos0(rbd::AW_Q) = 1.;
	updateKinematics(base_pos0, joint_pos,
					 Eigen::Vector6d::Zero(), joint_vel,
					 Eigen::Vector6d::Zero(), Eigen::VectorXd::Zero(fbs_->getJointDoF()));

	// Updating the frame Jacobians and kinematics
	updateJacobians(base_pos0, joint_pos);
	
	// Getting the joint acceleration
	getJointAcceleration(joint_acc, frame_acc);
}


void WholeBodyKinematics::getJointAcceleration(Eigen::VectorXd& joint_acc,
											   const Eigen::Vector6dMap& frame_acc)
{
	// Resizing the vector
	joint_acc = Eigen::VectorXd::Zero(fbs_->getJointDoF());

	// Getting the world to base transform
	unsigned int base_id = fbs_->getModel().getFrameId(fbs_->getFloatingBaseName());
	const se3::SE3 &w_X_b = fbs_->getData().oMf[base_id];

	// Getting the base acceleration
	Eigen::Vector6d base_acc;
	base_acc << fbs_->getData().a[base_id].angular(),
				fbs_->getData().a[base_id].linear();

	for (Eigen::Vector6dMap::const_iterator it = frame_acc.begin();
		it != frame_acc.end(); ++it) {
		// Getting the frame information
		std::string name = it->first;
		Eigen::Vector6d a_frame = it->second;
		a_frame << rbd::linearPart(a_frame), rbd::angularPart(a_frame);
		unsigned int id = fbs_->getModel().getFrameId(name);

		// Computing the transformation between the local to base coordinates of the frame
		const se3::SE3 &w_X_f = se3::SE3(fbs_->getData().oMf[id].rotation(),
										 Eigen::Vector3d::Zero());
		Eigen::Matrix4d b_H_f =
			w_X_b.toHomogeneousMatrix().inverse() * w_X_f.toHomogeneousMatrix();
		const se3::SE3 &b_X_f = 
			se3::SE3(b_H_f.block<3,3>(0,0), b_H_f.block<3,1>(0,3)); // (R,t)

		// Computing the frame Jacobian (actuated part) in the base frame
		Eigen::Matrix6x full_jac(6,fbs_->getModel().nv);
		Eigen::Matrix6x fixed_jac(6,fbs_->getModel().nv);
		full_jac.setZero();
		se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, full_jac);
		getFixedBaseJacobian(fixed_jac, full_jac);
		fixed_jac = b_X_f.toActionMatrix() * fixed_jac;

		// Computing the Jd*qd term expressed in the base frame
		// Since we set up the acceleration vector null, then the Jdot * qdot term
		// is equals to the accelerations
		Eigen::Vector6d jd_qd = getFrameAcceleration(name); 
		jd_qd << rbd::linearPart(jd_qd), rbd::angularPart(jd_qd);

		// Computing the branch joint acceleration
		Eigen::VectorXd branch_joint_acc =
			fbs_->getBranchState(math::pseudoInverse(fixed_jac) * (a_frame - jd_qd),
								 name);

		// Setting up the branch joint acceleration
		fbs_->setBranchState(joint_acc, branch_joint_acc, name);
	}
}


Eigen::Matrix6xMap
WholeBodyKinematics::computeJacobian(const Eigen::Vector7d& base_pos,
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
		Eigen::Matrix6x jac = getFrameJacobian(name);

		// Pinocchio defines floating joints as (linear, angular)^T which is
		// not consistent with our DWL standard, i.e. (angular, linear)^T
		if (fbs_->isFloatingBase()) {
			Eigen::MatrixXd copy_jac = jac.leftCols(6);
			jac.col(rbd::AX_V) = copy_jac.col(3);
			jac.col(rbd::AY_V) = copy_jac.col(4);
			jac.col(rbd::AZ_V) = copy_jac.col(5);
			jac.col(rbd::LX_V) = copy_jac.col(0);
			jac.col(rbd::LY_V) = copy_jac.col(1);
			jac.col(rbd::LZ_V) = copy_jac.col(2);
		}
		
		ordered_jac << jac.bottomRows<3>(), jac.topRows<3>();
		frame_jac[name] = ordered_jac;
	}

	return frame_jac;
}


Eigen::Matrix6x
WholeBodyKinematics::getFrameJacobian(const std::string& name)
{
	unsigned int id = fbs_->getModel().getFrameId(name);

	// Computing the transformation between the local to world coordinates of the frame
	const se3::SE3 &w_X_f = se3::SE3(fbs_->getData().oMf[id].rotation(),
									 Eigen::Vector3d::Zero());
	
	Eigen::Matrix6x J(6, fbs_->getTangentDim());
	J.setZero();
	se3::getFrameJacobian(fbs_->getModel(), fbs_->getData(), id, J);

	return w_X_f.toActionMatrix() * J;
}


void WholeBodyKinematics::getFloatingBaseJacobian(Eigen::Matrix6d& jacobian,
												  const Eigen::MatrixXd& full_jacobian)
{
	if (fbs_->isFloatingBase())
		jacobian = full_jacobian.leftCols(6);
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
