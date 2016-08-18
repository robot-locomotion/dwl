#include <dwl/model/WholeBodyKinematics.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics()
{

}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::modelFromURDFFile(std::string urdf_file,
											std::string system_file,
											bool info)
{
	modelFromURDFModel(urdf_model::fileToXml(urdf_file), system_file, info);
}


void WholeBodyKinematics::modelFromURDFModel(std::string urdf_model,
											 std::string system_file,
											 bool info)
{
	// Reseting the floating-base system information given an URDF model
	system_.resetFromURDFModel(urdf_model, system_file);

	// Getting the list of movable and fixed bodies
	rbd::getListOfBodies(body_id_, system_.getRBDModel());

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(system_.getRBDModel());

	// Computing the middle value for IK routines
	joint_pos_middle_ = Eigen::VectorXd::Zero(system_.getJointDoF());
	urdf_model::JointLimits joint_limits = system_.getJointLimits();
	for (urdf_model::JointLimits::iterator it = joint_limits.begin();
			it != joint_limits.end(); it++) {
		std::string name = it->first;
		double lower_limit = it->second.lower;
		double upper_limit = it->second.upper;

		joint_pos_middle_(system_.getJointId(name)) = (upper_limit + lower_limit) / 2;
	}
}


void WholeBodyKinematics::computeForwardKinematics(rbd::BodyVectorXd& op_pos,
												   const rbd::Vector6d& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   const rbd::BodySelector& body_set,
												   enum rbd::Component component,
												   enum TypeOfOrientation type)
{
	// Resizing the position vector
	int lin_vars = 0, ang_vars = 0;
	switch(component) {
	case rbd::Linear:
		ang_vars = 0;
		lin_vars = 3;
		break;
	case rbd::Angular:
		switch (type) {
		case RollPitchYaw:
			ang_vars = 3;
			break;
		case Quaternion:
			ang_vars = 4;
			break;
		case RotationMatrix:
			ang_vars = 0;
			break;
		}
		lin_vars = 0;
		break;
	case rbd::Full:
		switch (type) {
		case RollPitchYaw:
			ang_vars = 3;
			break;
		case Quaternion:
			ang_vars = 4;
			break;
		case RotationMatrix:
			ang_vars = 0;
			break;
		}
		lin_vars = 3;
		break;
	}

	Eigen::VectorXd body_pos(ang_vars + lin_vars);


	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			unsigned int body_id = body_id_.find(body_name)->second;

			Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);

			Eigen::Matrix3d rotation_mtx;
			switch (component) {
			case rbd::Linear:
				body_pos.segment<3>(0) =
						CalcBodyToBaseCoordinates(system_.getRBDModel(),
												  q, body_id,
												  Eigen::Vector3d::Zero(), true);
				break;
			case rbd::Angular:
				rotation_mtx =
						RigidBodyDynamics::CalcBodyWorldOrientation(system_.getRBDModel(),
																	q, body_id, false);
				switch (type) {
					case RollPitchYaw:
						body_pos.segment<3>(0) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						body_pos.segment<4>(0) = math::getQuaternion(rotation_mtx).coeffs();
						break;
					case RotationMatrix:
						break;
				}
				break;
			case rbd::Full:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(system_.getRBDModel(),
																		   q, body_id, false);
				switch (type) {
					case RollPitchYaw:
						body_pos.segment<3>(0) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						body_pos.segment<4>(0) = math::getQuaternion(rotation_mtx).coeffs();
						break;
					case RotationMatrix:
						break;
				}

				// Computing the linear component
				body_pos.segment<3>(ang_vars) =
						CalcBodyToBaseCoordinates(system_.getRBDModel(),
												  q, body_id,
												  Eigen::Vector3d::Zero(), true);
				break;
			}

			op_pos[body_name] = body_pos;
		}
	}
}


void WholeBodyKinematics::computeInverseKinematics(rbd::Vector6d& base_pos,
												   Eigen::VectorXd& joint_pos,
												   const rbd::BodyVector3d& op_pos,
												   const rbd::Vector6d& base_pos_init,
												   const Eigen::VectorXd& joint_pos_init,
												   double step_tol,
												   double lambda,
												   unsigned int max_iter)
{//TODO this routines has to consider more general cases, i.e. 6d operational position
	Eigen::VectorXd joint_pos_guess = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (joint_pos_init.size() == 0)
		joint_pos_guess = joint_pos_middle_;
	else
		joint_pos_guess = joint_pos_init;

	// Setting the desired body position for RBDL
	std::vector<unsigned int> body_id;
	std::vector<RigidBodyDynamics::Math::Vector3d> body_point;
	std::vector<RigidBodyDynamics::Math::Vector3d> target_pos;
	for (rbd::BodyVector3d::const_iterator body_iter = op_pos.begin();
			body_iter != op_pos.end();
			body_iter++)
	{
		std::string body_name = body_iter->first;
		if (body_id_.count(body_name) > 0) {
			body_id.push_back(body_id_.find(body_name)->second);
			body_point.push_back(RigidBodyDynamics::Math::Vector3d::Zero());
			target_pos.push_back((Eigen::Vector3d) op_pos.find(body_name)->second);
		}
	}

	// Converting the initial base position and joint position
	Eigen::VectorXd q_guess =
			system_.toGeneralizedJointState(base_pos_init, joint_pos_guess);

	// Computing the inverse kinematics
	Eigen::VectorXd q_res;
	RigidBodyDynamics::InverseKinematics(system_.getRBDModel(),
										 q_guess, body_id, body_point,
										 target_pos, q_res,
										 step_tol, lambda, max_iter);

	// Converting the base and joint positions
	system_.fromGeneralizedJointState(base_pos, joint_pos, q_res);
}


void WholeBodyKinematics::computeInverseKinematics(Eigen::VectorXd& joint_pos,
												   const rbd::BodyVector3d& op_pos,
												   const Eigen::VectorXd& joint_pos_init,
												   double step_tol,
												   double lambda,
												   unsigned int max_iter)
{
	// Setting up the guess point
	joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	if (joint_pos_init.size() == 0)
		joint_pos = joint_pos_middle_;
	else
		joint_pos = joint_pos_init;

	// Getting the end-effector names
	rbd::BodySelector body_names;
	std::vector<Eigen::Vector3d> target_pos;
	for (rbd::BodyVector3d::const_iterator contact_it = op_pos.begin();
			contact_it != op_pos.end(); contact_it++) {
		body_names.push_back(contact_it->first);
		target_pos.push_back(contact_it->second);
	}

	// Defining the residual error
	Eigen::VectorXd e = Eigen::VectorXd::Zero(3 * body_names.size());

	// Iterating until a satisfied the desired tolerance or reach the maximum
	// number of iterations
	Eigen::MatrixXd full_jac, fixed_jac, JJTe_lambda2_I;
	rbd::Vector6d base_pos = rbd::Vector6d::Zero();
	for (unsigned int k = 0; k < max_iter; k++) {
		// Computing the Jacobian
		computeJacobian(full_jac, base_pos, joint_pos, body_names, rbd::Linear);
		getFixedBaseJacobian(fixed_jac, full_jac);

		// Computing the forward kinematics
		rbd::BodyVectorXd fk_pos;
		computeForwardKinematics(fk_pos, base_pos, joint_pos, body_names, rbd::Linear);

		// Computing the error
		for (unsigned int f = 0; f < body_names.size(); f++) {
			e.segment<3>(3 * f) = target_pos[f] -
					(Eigen::Vector3d) fk_pos.find(body_names[f])->second;
		}

		// Computing the weighted fixed jacobian
		JJTe_lambda2_I = fixed_jac * fixed_jac.transpose() +
				lambda * lambda * Eigen::MatrixXd::Identity(e.size(), e.size());

		// Solving the linear system
		Eigen::VectorXd z;
		math::GaussianEliminationPivot(z, JJTe_lambda2_I, e);

		Eigen::VectorXd delta_theta = fixed_jac.transpose() * z; //math::pseudoInverse(fixed_jac) * e;
		joint_pos = joint_pos + delta_theta;
		if (delta_theta.norm() < step_tol)
			return;
	}
}


void WholeBodyKinematics::computeJointVelocity(Eigen::VectorXd& joint_vel,
											   const Eigen::VectorXd& joint_pos,
											   const rbd::BodyVectorXd& op_vel,
											   const rbd::BodySelector& body_set)
{
	// Computing the joint velocities per every body
	for (unsigned int f = 0; f < body_set.size(); f++) {
		std::string body_name = body_set[f];

		// Computing the joint velocity associated to the actual body
		rbd::BodyVectorXd::const_iterator vel_it = op_vel.find(body_name);
		if (vel_it != op_vel.end()) {
			Eigen::VectorXd body_vel = vel_it->second;

			// Computing the body jacobians
			Eigen::MatrixXd branch_jac;
			computeFixedJacobian(branch_jac, joint_pos, body_name, rbd::Linear);

			// Computing the branch joint velocity
			Eigen::VectorXd branch_joint_vel =
					math::pseudoInverse(branch_jac) * body_vel;

			// Setting up the branch joint velocity
			system_.setBranchState(joint_vel, branch_joint_vel, body_name);
		} else
			printf(YELLOW "Warning: the operational velocity of %s body was "
					"not defined\n" COLOR_RESET, body_name.c_str());
	}
}


void WholeBodyKinematics::computeJoinAcceleration(Eigen::VectorXd& joint_acc,
												  const Eigen::VectorXd& joint_pos,
												  const Eigen::VectorXd& joint_vel,
												  const rbd::BodyVectorXd& op_acc,
												  const rbd::BodySelector& body_set)
{
	// Computing the Jac_d*Qd
	dwl::rbd::BodyVectorXd jacd_qd;
	computeJdotQdot(jacd_qd,
					rbd::Vector6d::Zero(), joint_pos,
					rbd::Vector6d::Zero(), joint_vel,
					body_set, dwl::rbd::Linear);

	// Computing the joint accelerations per every body
	for (unsigned int f = 0; f < body_set.size(); f++) {
		std::string body_name = body_set[f];

		// Computing the joint acceleration associated to the actual body
		rbd::BodyVectorXd::const_iterator acc_it = op_acc.find(body_name);
		if (acc_it != op_acc.end()) {
			Eigen::VectorXd body_acc = acc_it->second;

			// Computing the body jacobians
			Eigen::MatrixXd branch_jac;
			computeFixedJacobian(branch_jac, joint_pos, body_name, rbd::Linear);

			// Computing the branch joint acceleration
			Eigen::VectorXd branch_joint_acc =
					math::pseudoInverse(branch_jac) * (body_acc -
							jacd_qd.find(body_name)->second);

			// Setting up the branch joint velocity
			system_.setBranchState(joint_acc, branch_joint_acc, body_name);
		} else
			printf(YELLOW "Warning: the operational acceleration of %s body was "
					"not defined\n" COLOR_RESET, body_name.c_str());
	}
}


void WholeBodyKinematics::computeJacobian(Eigen::MatrixXd& jacobian,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::BodySelector& body_set,
										  enum rbd::Component component)
{
	// Resizing the jacobian matrix
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		num_vars = 6;
		break;
	}

	// Computing the number of active end-effectors
	int num_body_set = getNumberOfActiveEndEffectors(body_set);

	jacobian.resize(num_vars * num_body_set, system_.getSystemDoF());
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int body_counter = 0;
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		int init_row = body_counter * num_vars;

		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			int body_id = body_id_.find(body_name)->second;

			Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);

			Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(6, system_.getSystemDoF()));
			if (body_counter == 0) {
				rbd::computePointJacobian(system_.getRBDModel(),
										  q, body_id,
										  Eigen::Vector3d::Zero(),
										  jac, true);
			} else {
				rbd::computePointJacobian(system_.getRBDModel(),
										  q, body_id,
										  Eigen::Vector3d::Zero(),
										  jac, false);
			}
			if (system_.isFullyFloatingBase()) {
				// RBDL defines floating joints as (linear, angular)^T which is
				// not consistent with our DWL standard, i.e. (angular, linear)^T
				Eigen::MatrixXd copy_jac = jac.block<6,6>(0,0);
				jac.block<6,3>(0,0) = copy_jac.rightCols(3);
				jac.block<6,3>(0,3) = copy_jac.leftCols(3);
			}

			switch(component) {
			case rbd::Linear:
				jacobian.block(init_row, 0, num_vars, system_.getSystemDoF()) =
						jac.block(3, 0, 3, system_.getSystemDoF());
				break;
			case rbd::Angular:
				jacobian.block(init_row, 0, num_vars, system_.getSystemDoF()) =
						jac.block(0, 0, 3, system_.getSystemDoF());
				break;
			case rbd::Full:
				jacobian.block(init_row, 0, num_vars, system_.getSystemDoF()) = jac;
				break;
			}
			++body_counter;
		}
	}
}


void WholeBodyKinematics::computeFixedJacobian(Eigen::MatrixXd& jacobian,
											   const Eigen::VectorXd& joint_pos,
											   const std::string& body_name,
											   enum rbd::Component component)
{
	// Resizing the jacobian matrix
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		num_vars = 6;
		break;
	}

	// Computing the full jacobian
	Eigen::MatrixXd full_jac;
	rbd::BodySelector body_set(1, body_name);
	computeJacobian(full_jac,
					rbd::Vector6d::Zero(), joint_pos,
					body_set, component);

	// Getting the position index and number of the dof of the branch
	unsigned int q_index, num_dof;
	system_.getBranch(q_index, num_dof, body_name);
	jacobian = full_jac.block(0, q_index, num_vars, num_dof);
}


void WholeBodyKinematics::getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
												  const Eigen::MatrixXd& full_jacobian)
{
	if (system_.getTypeOfDynamicSystem() == FloatingBase ||
			system_.getTypeOfDynamicSystem() == ConstrainedFloatingBase)
		jacobian = full_jacobian.leftCols<6>();
	else if (system_.getTypeOfDynamicSystem() == VirtualFloatingBase) {
		jacobian = Eigen::MatrixXd::Zero(full_jacobian.rows(), system_.getFloatingBaseDoF());

		// Adding the first n column associated with the floating-base joints
		for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
			rbd::Coords6d base_coord = rbd::Coords6d(base_idx);
			FloatingBaseJoint base_joint = system_.getFloatingBaseJoint(base_coord);

			if (base_joint.active)
				jacobian.col(base_joint.id) = full_jacobian.col(base_joint.id);
		}
	} else {
		printf(YELLOW "Warning: this is a fixed-base robot\n" COLOR_RESET);
		jacobian = Eigen::MatrixXd::Zero(0,0);
	}
}


void WholeBodyKinematics::getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
											   const Eigen::MatrixXd& full_jacobian)
{
	if (system_.getTypeOfDynamicSystem() == FloatingBase ||
			system_.getTypeOfDynamicSystem() == ConstrainedFloatingBase)
		jacobian = full_jacobian.rightCols(system_.getJointDoF());
	else if (system_.getTypeOfDynamicSystem() == VirtualFloatingBase)
		jacobian = full_jacobian.rightCols(system_.getJointDoF());
	else
		jacobian = full_jacobian;
}


void WholeBodyKinematics::computeVelocity(rbd::BodyVectorXd& op_vel,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  const rbd::BodySelector& body_set,
										  enum rbd::Component component)
{
	// Resizing the velocity vector
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		num_vars = 6;
		break;
	}
	Eigen::VectorXd body_vel(num_vars);


	// Adding the velocity only for the active end-effectors
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			int body_id = body_id_.find(body_name)->second;

			Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);
			Eigen::VectorXd q_dot = system_.toGeneralizedJointState(base_vel, joint_vel);

			// Computing the point velocity
			rbd::Vector6d point_vel =
					rbd::computePointVelocity(system_.getRBDModel(),
											  q, q_dot, body_id,
											  Eigen::Vector3d::Zero(), true);
			switch (component) {
			case rbd::Linear:
				body_vel.segment<3>(0) = rbd::linearPart(point_vel);
				break;
			case rbd::Angular:
				body_vel.segment<3>(0) = rbd::angularPart(point_vel);
				break;
			case rbd::Full:
				body_vel.segment<6>(0) = point_vel;
				break;
			}

			op_vel[body_name] = body_vel;
		}
	}
}


void WholeBodyKinematics::computeAcceleration(rbd::BodyVectorXd& op_acc,
											  const rbd::Vector6d& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  const rbd::Vector6d& base_vel,
											  const Eigen::VectorXd& joint_vel,
											  const rbd::Vector6d& base_acc,
											  const Eigen::VectorXd& joint_acc,
											  const rbd::BodySelector& body_set,
											  enum rbd::Component component)
{
	// Resizing the velocity vector
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		num_vars = 6;
		break;
	}

	Eigen::VectorXd body_acc(num_vars);

	// Adding the velocity only for the active end-effectors
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			unsigned int body_id = body_id_.find(body_name)->second;

			Eigen::VectorXd q = system_.toGeneralizedJointState(base_pos, joint_pos);
			Eigen::VectorXd q_dot = system_.toGeneralizedJointState(base_vel, joint_vel);
			Eigen::VectorXd q_ddot = system_.toGeneralizedJointState(base_acc, joint_acc);

			// Computing the point acceleration
			rbd::Vector6d point_acc =
					rbd::computePointAcceleration(system_.getRBDModel(),
												  q, q_dot, q_ddot,
												  body_id,
												  Eigen::Vector3d::Zero(), true);
			switch (component) {
			case rbd::Linear:
				body_acc.segment<3>(0) = rbd::linearPart(point_acc);
				break;
			case rbd::Angular:
				body_acc.segment<3>(0) = rbd::angularPart(point_acc);
				break;
			case rbd::Full:
				body_acc.segment<6>(0) = point_acc;
				break;
			}

			op_acc[body_name] = body_acc;
		}
	}
}


void WholeBodyKinematics::computeJdotQdot(rbd::BodyVectorXd& jacd_qd,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  const rbd::BodySelector& body_set,
										  enum rbd::Component component)
{
	rbd::BodyVectorXd op_vel, op_acc;
	computeAcceleration(op_acc,
						base_pos, joint_pos,
						base_vel, joint_vel,
						rbd::Vector6d::Zero(), Eigen::VectorXd::Zero(system_.getJointDoF()),
						body_set, component);

	// Resizing the acceleration contribution vector
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		computeVelocity(op_vel,
						base_pos, joint_pos,
						base_vel, joint_vel,
						body_set);
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		computeVelocity(op_vel,
						base_pos, joint_pos,
						base_vel, joint_vel,
						body_set);
		num_vars = 6;
		break;
	}

	Eigen::VectorXd body_jacd_qd(num_vars);

	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			switch (component) {
			case rbd::Linear: {
				// Computing the point velocity and its angular and linear
				// components
				rbd::Vector6d point_vel = op_vel[body_name];
				Eigen::Vector3d ang_vel, lin_vel;
				ang_vel = rbd::angularPart(point_vel);
				lin_vel = rbd::linearPart(point_vel);

				// Computing the JdQd for current point
				body_jacd_qd.segment<3>(0) =
						op_acc[body_name] + ang_vel.cross(lin_vel);
				break;
			} case rbd::Angular: {
				// Computing the JdQd for current point
				body_jacd_qd.segment<3>(0) = op_acc[body_name];
				break;
			} case rbd::Full: {
				// Computing the point velocity and its angular and linear
				// components
				rbd::Vector6d point_vel = op_vel[body_name];
				Eigen::Vector3d ang_vel, lin_vel;
				ang_vel = rbd::angularPart(point_vel);
				lin_vel = rbd::linearPart(point_vel);

				// Computing the JdQd for current point
				rbd::Vector6d point_acc = op_acc[body_name];
				body_jacd_qd.segment<3>(rbd::AX) = rbd::angularPart(point_acc);
				body_jacd_qd.segment<3>(rbd::LX) =
						rbd::linearPart(point_acc) + ang_vel.cross(lin_vel);
				break;}
			}

			jacd_qd[body_name] = body_jacd_qd;
		}
	}
}


const FloatingBaseSystem& WholeBodyKinematics::getFloatingBaseSystem() const
{
	return system_;
}


int WholeBodyKinematics::getNumberOfActiveEndEffectors(const rbd::BodySelector& body_set)
{
	int num_body_set = 0;
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			++num_body_set;
		} else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n"
					COLOR_RESET, body_name.c_str());
	}

	return num_body_set;
}

} //@namespace model
} //@namespace dwl
