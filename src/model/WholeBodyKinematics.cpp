#include <model/WholeBodyKinematics.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics() : reduced_base_(NULL)
{

}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::modelFromURDF(std::string model_file, struct rbd::ReducedFloatingBase* reduce_base, bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);

	reduced_base_ = reduce_base;

	// Adding the fixed body in the end-effector list
	for (unsigned int it = 0; it < robot_model_.mFixedBodies.size(); it++) {
		unsigned int body_id = it + robot_model_.fixed_body_discriminator;
		std::string body_name = robot_model_.GetBodyName(body_id);

		body_id_[body_name] = body_id;
		full_effector_set_.push_back(body_name);
	}

	if (info) {
		std::cout << "Degree of freedom overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot_model_);

		std::cout << "Body origins overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(robot_model_);

		std::cout << "Model Hierarchy:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(robot_model_);
	}
}


void WholeBodyKinematics::computeForwardKinematics(Eigen::VectorXd& op_pos,
												   const rbd::Vector6d& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   enum rbd::Component component,
												   enum TypeOfOrientation type)
{
	// Computing the forward kinematics for all end-effectors
	rbd::EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeForwardKinematics(op_pos, base_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeForwardKinematics(Eigen::VectorXd& op_pos,
												   const rbd::Vector6d& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   rbd::EndEffectorSelector effector_set,
												   enum rbd::Component component,
												   enum TypeOfOrientation type)
{
	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	// Resizing the whole-body operation position vector
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
	op_pos.resize(num_effector_set * (ang_vars + lin_vars));

	int effector_counter = 0;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = *effector_iter;
		if (body_id_.count(effector_name) > 0) {
			unsigned int effector_id = body_id_.find(effector_name)->second;
			int init_col = effector_counter * (ang_vars + lin_vars);

			Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos, reduced_base_);

			Eigen::Matrix3d rotation_mtx;
			switch (component) {
			case rbd::Linear:
				op_pos.segment(init_col, lin_vars) =
						CalcBodyToBaseCoordinates(robot_model_, q, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			case rbd::Angular:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, effector_id, false);
				switch (type) {
					case RollPitchYaw:
						op_pos.segment(init_col, ang_vars) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						op_pos.segment(init_col, ang_vars) = math::getQuaternion(rotation_mtx).coeffs();
						break;
					case RotationMatrix:
						break;
				}
				break;
			case rbd::Full:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, effector_id, false);
				switch (type) {
					case RollPitchYaw:
						op_pos.segment(init_col, ang_vars) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						op_pos.segment(init_col, ang_vars) = math::getQuaternion(rotation_mtx).coeffs();
						break;
					case RotationMatrix:
						break;
				}

				// Computing the linear component
				op_pos.segment(init_col + ang_vars, lin_vars) =
						CalcBodyToBaseCoordinates(robot_model_, q, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			++effector_counter;
		}
	}
}


void WholeBodyKinematics::computeInverseKinematics(rbd::Vector6d& base_pos,
												   Eigen::VectorXd& joint_pos,
												   const rbd::Vector6d& base_pos_init,
												   const Eigen::VectorXd& joint_pos_init,
												   rbd::EndEffectorPosition op_pos,
												   double step_tol,
												   double lambda,
												   unsigned int max_iter)
{
	// Computing the inverse kinematics for all end-effectors
	rbd::EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeInverseKinematics(base_pos, joint_pos, base_pos_init, joint_pos_init, op_pos, effector_set);
}


void WholeBodyKinematics::computeInverseKinematics(rbd::Vector6d& base_pos,
												   Eigen::VectorXd& joint_pos,
												   const rbd::Vector6d& base_pos_init,
												   const Eigen::VectorXd& joint_pos_init,
												   rbd::EndEffectorPosition op_pos,
												   rbd::EndEffectorSelector effector_set,
												   double step_tol,
												   double lambda,
												   unsigned int max_iter)
{
	int effector_counter = 0;
	std::vector<unsigned int> body_id;
	std::vector<RigidBodyDynamics::Math::Vector3d> body_point;
	std::vector<RigidBodyDynamics::Math::Vector3d> target_pos;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = *effector_iter;
		if (body_id_.count(effector_name) > 0) {
			body_id.push_back(body_id_.find(effector_name)->second);
			body_point.push_back(RigidBodyDynamics::Math::Vector3d::Zero());
			target_pos.push_back((Eigen::Vector3d) op_pos.find(effector_name)->second);

			++effector_counter;
		}
	}


	Eigen::VectorXd q_init = rbd::toGeneralizedJointState(robot_model_, base_pos_init, joint_pos_init, reduced_base_);
	Eigen::VectorXd q_res;
	RigidBodyDynamics::InverseKinematics(robot_model_, q_init, body_id, body_point, target_pos,
										q_res, step_tol, lambda, max_iter);

	rbd::fromGeneralizedJointState(robot_model_, base_pos, joint_pos, q_res, reduced_base_);
}


void WholeBodyKinematics::computeJacobian(Eigen::MatrixXd& jacobian,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  enum rbd::Component component)
{
	// Computing the jacobian for all end-effectors
	rbd::EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeJacobian(jacobian, base_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeJacobian(Eigen::MatrixXd& jacobian,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  rbd::EndEffectorSelector effector_set,
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
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	jacobian.resize(num_vars * num_effector_set, robot_model_.dof_count);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		int init_row = effector_counter * num_vars;

		std::string effector_name = *effector_iter;
		if (body_id_.count(effector_name) > 0) {
			int effector_id = body_id_.find(effector_name)->second;

			Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos, reduced_base_);

			Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(6, robot_model_.dof_count));
			rbd::computePointJacobian(robot_model_, q, effector_id, Eigen::VectorXd::Zero(robot_model_.dof_count), jac, true);

			switch(component) {
			case rbd::Linear:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac.block(3,0,6,robot_model_.dof_count);
				break;
			case rbd::Angular:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac.block(0,0,3,robot_model_.dof_count);
				break;
			case rbd::Full:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac;
				break;
			}
			++effector_counter;
		}
	}
}


void WholeBodyKinematics::getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
												  const Eigen::MatrixXd& full_jacobian)
{
	if (rbd::isFloatingBaseRobot(robot_model_))
		jacobian = full_jacobian.leftCols<6>();
	else if (rbd::isVirtualFloatingBaseRobot(reduced_base_)) {
		unsigned int num_virtual_jnts = reduced_base_->getFloatingBaseDOF();
		jacobian = Eigen::MatrixXd::Zero(full_jacobian.rows(), num_virtual_jnts);

		if (reduced_base_->TX.active)
			jacobian.col(reduced_base_->TX.id) = full_jacobian.col(reduced_base_->TX.id);
		if (reduced_base_->TY.active)
			jacobian.col(reduced_base_->TY.id) = full_jacobian.col(reduced_base_->TY.id);
		if (reduced_base_->TZ.active)
			jacobian.col(reduced_base_->TZ.id) = full_jacobian.col(reduced_base_->TZ.id);
		if (reduced_base_->RX.active)
			jacobian.col(reduced_base_->RX.id) = full_jacobian.col(reduced_base_->RX.id);
		if (reduced_base_->RY.active)
			jacobian.col(reduced_base_->RY.id) = full_jacobian.col(reduced_base_->RY.id);
		if (reduced_base_->RZ.active)
			jacobian.col(reduced_base_->RZ.id) = full_jacobian.col(reduced_base_->RZ.id);
	} else {
		printf(YELLOW "Warning: this is a fixed-base robot\n" COLOR_RESET);
		jacobian = Eigen::MatrixXd::Zero(0,0);
	}
}


void WholeBodyKinematics::getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
											   const Eigen::MatrixXd& full_jacobian)
{
	if (rbd::isFloatingBaseRobot(robot_model_))
		jacobian = full_jacobian.rightCols(robot_model_.dof_count - 6);
	else if (rbd::isVirtualFloatingBaseRobot(reduced_base_)) {
		unsigned int num_virtual_jnts = reduced_base_->getFloatingBaseDOF();
		jacobian = full_jacobian.rightCols(robot_model_.dof_count - num_virtual_jnts);
	} else
		jacobian = full_jacobian;
}


void WholeBodyKinematics::computeVelocity(Eigen::VectorXd& op_vel,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  enum rbd::Component component)
{
	// Computing the velocity for all end-effectors
	rbd::EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeVelocity(op_vel, base_pos, joint_pos, base_vel, joint_vel, effector_set, component);
}


void WholeBodyKinematics::computeVelocity(Eigen::VectorXd& op_vel,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  rbd::EndEffectorSelector effector_set,
										  enum rbd::Component component)
{
	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

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
	op_vel.resize(num_effector_set * num_vars);

	// Adding the velocity only for the active end-effectors
	int effector_counter = 0;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = *effector_iter;
		if (body_id_.count(effector_name) > 0) {
			int effector_id = body_id_.find(effector_name)->second;
			int init_col = effector_counter * num_vars;

			Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos, reduced_base_);
			Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel, reduced_base_);

			switch (component) {
			case rbd::Linear:
				op_vel.segment(init_col, num_vars) =
						rbd::computePointVelocity(robot_model_, q, q_dot, effector_id, Eigen::Vector3d::Zero(), true).tail(3);
				break;
			case rbd::Angular:
				op_vel.segment(init_col, num_vars) =
						rbd::computePointVelocity(robot_model_, q, q_dot, effector_id, Eigen::Vector3d::Zero(), true).head(3);
				break;
			case rbd::Full:
				op_vel.segment(init_col, num_vars) =
						rbd::computePointVelocity(robot_model_, q, q_dot, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			++effector_counter;
		}
	}
}


void WholeBodyKinematics::computeAcceleration(Eigen::VectorXd& op_acc,
											  const rbd::Vector6d& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  const rbd::Vector6d& base_vel,
											  const Eigen::VectorXd& joint_vel,
											  const rbd::Vector6d& base_acc,
											  const Eigen::VectorXd& joint_acc,
											  enum rbd::Component component)
{
	// Computing the acceleration for all end-effectors
	rbd::EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeAcceleration(op_acc, base_pos, joint_pos, base_vel, joint_vel,
						base_acc, joint_acc, effector_set, component);
}


void WholeBodyKinematics::computeAcceleration(Eigen::VectorXd& op_acc,
											  const rbd::Vector6d& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  const rbd::Vector6d& base_vel,
											  const Eigen::VectorXd& joint_vel,
											  const rbd::Vector6d& base_acc,
											  const Eigen::VectorXd& joint_acc,
											  rbd::EndEffectorSelector effector_set,
											  enum rbd::Component component)
{
	// Computing the number of active end-effectors
	int num_effector_set = 3;//getNumberOfActiveEndEffectors(effector_set);

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
	op_acc.resize(num_effector_set * num_vars);

	// Adding the velocity only for the active end-effectors
	int effector_counter = 0;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = *effector_iter;
		if (body_id_.count(effector_name) > 0) {
			unsigned int effector_id = body_id_.find(effector_name)->second;
			int init_col = effector_counter * num_vars;

			Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos, reduced_base_);
			Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel, reduced_base_);
			Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc, reduced_base_);

			switch (component) {
			case rbd::Linear:
				op_acc.segment(init_col, num_vars) =
						rbd::computePointAcceleration(robot_model_, q, q_dot, q_ddot, effector_id, Eigen::Vector3d::Zero(), true).tail(3);
				break;
			case rbd::Angular:
				op_acc.segment(init_col, num_vars) =
						rbd::computePointAcceleration(robot_model_, q, q_dot, q_ddot, effector_id, Eigen::Vector3d::Zero(), true).head(3);
				break;
			case rbd::Full:
				op_acc.segment(init_col, num_vars) =
						rbd::computePointAcceleration(robot_model_, q, q_dot, q_ddot, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			++effector_counter;
		}
	}
}


void WholeBodyKinematics::computeJdotQdot(Eigen::VectorXd& jacd_qd,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  enum rbd::Component component)
{
	// Computing the forward kinematics for all end-effectors
	rbd::EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeJdotQdot(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, effector_set, component);
}


void WholeBodyKinematics::computeJdotQdot(Eigen::VectorXd& jacd_qd,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  rbd::EndEffectorSelector effector_set,
										  enum rbd::Component component)
{
	// Getting the floating-base dof
	unsigned int floating_base_dof = rbd::getFloatingBaseDOF(robot_model_, reduced_base_);

	Eigen::VectorXd op_vel, op_acc;
	computeAcceleration(op_acc, base_pos, joint_pos,
						base_vel, joint_vel,
						rbd::Vector6d::Zero(), Eigen::VectorXd::Zero(robot_model_.dof_count - floating_base_dof),
						effector_set, component);

	// Resizing the acceleration contribution vector
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		computeVelocity(op_vel, base_pos, joint_pos, base_vel, joint_vel, effector_set);
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		computeVelocity(op_vel, base_pos, joint_pos, base_vel, joint_vel, effector_set);
		num_vars = 6;
		break;
	}
	jacd_qd.resize(num_effector_set * num_vars);

	for (int i = 0; i < num_effector_set; i++) {
		switch (component) {
		case rbd::Linear: {
			Eigen::Vector3d ang_vel, lin_vel;
			ang_vel = op_vel.segment(i * num_vars, 3);
			lin_vel = op_vel.segment(i * num_vars + 3, 3);
			jacd_qd.segment(i * num_vars, num_vars) = op_acc.segment(i * num_vars, num_vars) + ang_vel.cross(lin_vel);
			break;
		} case rbd::Angular: {
			jacd_qd.segment(i * num_vars, num_vars) = op_acc.segment(i * num_vars, num_vars);
			break;
		} case rbd::Full: {
			Eigen::Vector3d ang_vel, lin_vel;
			ang_vel = op_vel.segment(i * num_vars, 3);
			lin_vel = op_vel.segment(i * num_vars + 3, 3);
			jacd_qd.segment(i * num_vars, 3) = op_acc.segment(i * num_vars, 3);
			jacd_qd.segment(i * num_vars + 3, 3) = op_acc.segment(i * num_vars + 3, 3) + ang_vel.cross(lin_vel);
			break;}
		}
	}
}


int WholeBodyKinematics::getNumberOfActiveEndEffectors(rbd::EndEffectorSelector effector_set)
{
	int num_effector_set = 0;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = *effector_iter;
		if (body_id_.count(effector_name) > 0) {
			++num_effector_set;
		} else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, effector_name.c_str());
	}

	return num_effector_set;
}


void WholeBodyKinematics::activeAllEndEffector(rbd::EndEffectorSelector& effector_set)
{
	effector_set = full_effector_set_;
}

} //@namespace model
} //@namespace dwl
