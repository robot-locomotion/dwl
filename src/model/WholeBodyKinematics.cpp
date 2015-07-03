#include <model/WholeBodyKinematics.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics() : type_of_system_(rbd::FixedBase), reduced_base_(NULL)
{

}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::modelFromURDFFile(std::string model_file,
											struct rbd::ReducedFloatingBase* reduced_base,
											bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);

	reduced_base_ = reduced_base;

	// Getting the list of movable and fixed bodies
	rbd::getListOfBodies(body_id_, robot_model_);

	// Getting the type of dynamic system
	rbd::getTypeOfDynamicSystem(type_of_system_, robot_model_, reduced_base);

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(robot_model_);
}


void WholeBodyKinematics::modelFromURDFModel(std::string urdf_model,
											 struct rbd::ReducedFloatingBase* reduced_base,
											 bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromString(urdf_model.c_str(), &robot_model_, false);

	reduced_base_ = reduced_base;

	// Getting the list of movable and fixed bodies
	rbd::getListOfBodies(body_id_, robot_model_);

	// Getting the type of dynamic system
	rbd::getTypeOfDynamicSystem(type_of_system_, robot_model_, reduced_base);

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(robot_model_);
}


void WholeBodyKinematics::computeForwardKinematics(Eigen::VectorXd& op_pos,
												   const rbd::Vector6d& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   const rbd::BodySelector& body_set,
												   enum rbd::Component component,
												   enum TypeOfOrientation type)
{
	// Computing the number of active end-effectors
	int num_body_set = getNumberOfActiveEndEffectors(body_set);

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
	op_pos.resize(num_body_set * (ang_vars + lin_vars));

	int body_counter = 0;
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			unsigned int body_id = body_id_.find(body_name)->second;
			int init_col = body_counter * (ang_vars + lin_vars);

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, type_of_system_, reduced_base_);

			Eigen::Matrix3d rotation_mtx;
			switch (component) {
			case rbd::Linear:
				op_pos.segment<3>(init_col) =
						CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
				break;
			case rbd::Angular:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, body_id, false);
				switch (type) {
					case RollPitchYaw:
						op_pos.segment<3>(init_col) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						op_pos.segment<4>(init_col) = math::getQuaternion(rotation_mtx).coeffs();
						break;
					case RotationMatrix:
						break;
				}
				break;
			case rbd::Full:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, body_id, false);
				switch (type) {
					case RollPitchYaw:
						op_pos.segment<3>(init_col) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						op_pos.segment<4>(init_col) = math::getQuaternion(rotation_mtx).coeffs();
						break;
					case RotationMatrix:
						break;
				}

				// Computing the linear component
				op_pos.segment<3>(init_col + ang_vars) =
						CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			++body_counter;
		}
	}
}


void WholeBodyKinematics::computeInverseKinematics(rbd::Vector6d& base_pos,
												   Eigen::VectorXd& joint_pos,
												   const rbd::Vector6d& base_pos_init,
												   const Eigen::VectorXd& joint_pos_init,
												   const rbd::BodyPosition& op_pos,
												   double step_tol,
												   double lambda,
												   unsigned int max_iter)
{
	// Setting the desired body position for RBDL
	std::vector<unsigned int> body_id;
	std::vector<RigidBodyDynamics::Math::Vector3d> body_point;
	std::vector<RigidBodyDynamics::Math::Vector3d> target_pos;
	for (rbd::BodyPosition::const_iterator body_iter = op_pos.begin();
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
	Eigen::VectorXd q_init = rbd::toGeneralizedJointState(base_pos_init, joint_pos_init, type_of_system_, reduced_base_);

	// Computing the inverse kinematics
	Eigen::VectorXd q_res;
	RigidBodyDynamics::InverseKinematics(robot_model_, q_init, body_id, body_point, target_pos,
										 q_res, step_tol, lambda, max_iter);

	// Converting the base and joint positions
	rbd::fromGeneralizedJointState(base_pos, joint_pos, q_res, type_of_system_, reduced_base_);
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

	jacobian.resize(num_vars * num_body_set, robot_model_.dof_count);
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

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, type_of_system_, reduced_base_);

			Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(6, robot_model_.dof_count));
			rbd::computePointJacobian(robot_model_, q, body_id, Eigen::VectorXd::Zero(robot_model_.dof_count), jac, true);

			switch(component) {
			case rbd::Linear:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac.block(3, 0, 6, robot_model_.dof_count);
				break;
			case rbd::Angular:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac.block(0, 0, 3, robot_model_.dof_count);
				break;
			case rbd::Full:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac;
				break;
			}
			++body_counter;
		}
	}
}


void WholeBodyKinematics::getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
												  const Eigen::MatrixXd& full_jacobian)
{
	if (type_of_system_ == rbd::FloatingBase || type_of_system_ == rbd::ConstrainedFloatingBase)
		jacobian = full_jacobian.leftCols<6>();
	else if (type_of_system_ == rbd::VirtualFloatingBase) {
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
	if (type_of_system_ == rbd::FloatingBase || type_of_system_ == rbd::ConstrainedFloatingBase)
		jacobian = full_jacobian.rightCols(robot_model_.dof_count - 6);
	else if (type_of_system_ == rbd::VirtualFloatingBase) {
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
										  const rbd::BodySelector& body_set,
										  enum rbd::Component component)
{
	// Computing the number of active end-effectors
	int num_body_set = getNumberOfActiveEndEffectors(body_set);

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
	op_vel.resize(num_body_set * num_vars);

	// Adding the velocity only for the active end-effectors
	int body_counter = 0;
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			int body_id = body_id_.find(body_name)->second;
			int init_col = body_counter * num_vars;

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, type_of_system_, reduced_base_);
			Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(base_vel, joint_vel, type_of_system_, reduced_base_);

			// Computing the point velocity
			rbd::Vector6d point_vel = rbd::computePointVelocity(robot_model_, q, q_dot, body_id, Eigen::Vector3d::Zero(), true);
			switch (component) {
			case rbd::Linear:
				op_vel.segment<3>(init_col) = rbd::linearPart(point_vel);
				break;
			case rbd::Angular:
				op_vel.segment<3>(init_col) = rbd::angularPart(point_vel);
				break;
			case rbd::Full:
				op_vel.segment<6>(init_col) = point_vel;
				break;
			}

			++body_counter;
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
											  const rbd::BodySelector& body_set,
											  enum rbd::Component component)
{
	// Computing the number of active end-effectors
	int num_body_set = getNumberOfActiveEndEffectors(body_set);

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
	op_acc.resize(num_body_set * num_vars);

	// Adding the velocity only for the active end-effectors
	int body_counter = 0;
	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			unsigned int body_id = body_id_.find(body_name)->second;
			int init_col = body_counter * num_vars;

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, type_of_system_, reduced_base_);
			Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(base_vel, joint_vel, type_of_system_, reduced_base_);
			Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(base_acc, joint_acc, type_of_system_, reduced_base_);

			// Computing the point acceleration
			rbd::Vector6d point_acc = rbd::computePointAcceleration(robot_model_, q, q_dot, q_ddot, body_id, Eigen::Vector3d::Zero(), true);
			switch (component) {
			case rbd::Linear:
				op_acc.segment<3>(init_col) = rbd::linearPart(point_acc);
				break;
			case rbd::Angular:
				op_acc.segment<3>(init_col) = rbd::angularPart(point_acc);
				break;
			case rbd::Full:
				op_acc.segment<6>(init_col) = point_acc;
				break;
			}

			++body_counter;
		}
	}
}


void WholeBodyKinematics::computeJdotQdot(Eigen::VectorXd& jacd_qd,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  const rbd::BodySelector& body_set,
										  enum rbd::Component component)
{
	// Getting the floating-base dof
	unsigned int floating_base_dof = rbd::getFloatingBaseDOF(robot_model_, reduced_base_);

	Eigen::VectorXd op_vel, op_acc;
	computeAcceleration(op_acc, base_pos, joint_pos,
						base_vel, joint_vel,
						rbd::Vector6d::Zero(), Eigen::VectorXd::Zero(robot_model_.dof_count - floating_base_dof),
						body_set, component);

	// Resizing the acceleration contribution vector
	int num_body_set = getNumberOfActiveEndEffectors(body_set);
	int num_vars = 0;
	switch (component) {
	case rbd::Linear:
		computeVelocity(op_vel, base_pos, joint_pos, base_vel, joint_vel, body_set);
		num_vars = 3;
		break;
	case rbd::Angular:
		num_vars = 3;
		break;
	case rbd::Full:
		computeVelocity(op_vel, base_pos, joint_pos, base_vel, joint_vel, body_set);
		num_vars = 6;
		break;
	}
	jacd_qd.resize(num_body_set * num_vars);

	for (int i = 0; i < num_body_set; i++) {
		switch (component) {
		case rbd::Linear: {
			// Computing the point velocity and its angular and linear components
			rbd::Vector6d point_vel = op_vel.segment<6>(i * num_vars);
			Eigen::Vector3d ang_vel, lin_vel;
			ang_vel = rbd::angularPart(point_vel);
			lin_vel = rbd::linearPart(point_vel);

			// Computing the JdQd for current point
			jacd_qd.segment<3>(i * num_vars) = op_acc.segment<3>(i * num_vars) + ang_vel.cross(lin_vel);
			break;
		} case rbd::Angular: {
			// Computing the JdQd for current point
			jacd_qd.segment<3>(i * num_vars + rbd::AX) = op_acc.segment<3>(i * num_vars);
			break;
		} case rbd::Full: {
			// Computing the point velocity and its angular and linear components
			rbd::Vector6d point_vel = op_vel.segment<6>(i * num_vars);
			Eigen::Vector3d ang_vel, lin_vel;
			ang_vel = rbd::angularPart(point_vel);
			lin_vel = rbd::linearPart(point_vel);

			// Computing the JdQd for current point
			jacd_qd.segment<3>(i * num_vars + rbd::AX) = op_acc.segment<3>(i * num_vars + rbd::AX);
			jacd_qd.segment<3>(i * num_vars + rbd::LX) = op_acc.segment<3>(i * num_vars + rbd::LX) + ang_vel.cross(lin_vel);
			break;}
		}
	}
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
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, body_name.c_str());
	}

	return num_body_set;
}

} //@namespace model
} //@namespace dwl
