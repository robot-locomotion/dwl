#include <model/WholeBodyKinematics.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics() : system_(NULL)
{

}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::modelFromURDFFile(std::string filename,
											struct rbd::FloatingBaseSystem* system,
											bool info)
{
	// Reading the file
	std::ifstream model_file(filename.c_str());
	if (!model_file) {
		std::cerr << "Error opening file '" << filename << "'." << std::endl;
		abort();
	}

	// Reserving memory for the contents of the file
	std::string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
			std::istreambuf_iterator<char>());
	model_file.close();

	modelFromURDFModel(model_xml_string, system, info);
}


void WholeBodyKinematics::modelFromURDFModel(std::string urdf_model,
											 struct rbd::FloatingBaseSystem* system,
											 bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromString(urdf_model.c_str(), &robot_model_, false);

	system_ = system;

	// Setting the number of joints
	unsigned int num_joints = robot_model_.dof_count - system_->getFloatingBaseDOF();
	system_->setJointDOF(num_joints);

	// Getting and setting the type of dynamic system
	enum rbd::TypeOfSystem type_of_system;
	rbd::getTypeOfDynamicSystem(type_of_system, robot_model_, system_);
	system_->setTypeOfDynamicSystem(type_of_system);

	// Getting the list of movable and fixed bodies
	rbd::getListOfBodies(body_id_, robot_model_);

	// Printing the information of the rigid-body system
	if (info)
		rbd::printModelInfo(robot_model_);
}


void WholeBodyKinematics::computeForwardKinematics(rbd::BodyVector& op_pos,
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

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, system_);

			Eigen::Matrix3d rotation_mtx;
			switch (component) {
			case rbd::Linear:
				body_pos.segment<3>(0) =
						CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
				break;
			case rbd::Angular:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, body_id, false);
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
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, body_id, false);
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
						CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			op_pos[body_name] = body_pos;
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
	Eigen::VectorXd q_init = rbd::toGeneralizedJointState(base_pos_init, joint_pos_init, system_);

	// Computing the inverse kinematics
	Eigen::VectorXd q_res;
	RigidBodyDynamics::InverseKinematics(robot_model_, q_init, body_id, body_point, target_pos,
										 q_res, step_tol, lambda, max_iter);

	// Converting the base and joint positions
	rbd::fromGeneralizedJointState(base_pos, joint_pos, q_res, system_);
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

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, system_);

			Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(6, robot_model_.dof_count));
			rbd::computePointJacobian(robot_model_, q, body_id, Eigen::VectorXd::Zero(robot_model_.dof_count), jac, true);

			switch(component) {
			case rbd::Linear:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) =
						jac.block(3, 0, 6, robot_model_.dof_count);
				break;
			case rbd::Angular:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) =
						jac.block(0, 0, 3, robot_model_.dof_count);
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
	enum rbd::TypeOfSystem type_of_system = system_->getTypeOfDynamicSystem();
	if (type_of_system == rbd::FloatingBase || type_of_system == rbd::ConstrainedFloatingBase)
		jacobian = full_jacobian.leftCols<6>();
	else if (type_of_system == rbd::VirtualFloatingBase) {
		unsigned int num_virtual_jnts = system_->getFloatingBaseDOF();
		jacobian = Eigen::MatrixXd::Zero(full_jacobian.rows(), num_virtual_jnts);

		if (system_->AX.active)
			jacobian.col(system_->AX.id) = full_jacobian.col(system_->AX.id);
		if (system_->AY.active)
			jacobian.col(system_->AY.id) = full_jacobian.col(system_->AY.id);
		if (system_->AZ.active)
			jacobian.col(system_->AZ.id) = full_jacobian.col(system_->AZ.id);
		if (system_->LX.active)
			jacobian.col(system_->LX.id) = full_jacobian.col(system_->LX.id);
		if (system_->LY.active)
			jacobian.col(system_->LY.id) = full_jacobian.col(system_->LY.id);
		if (system_->LZ.active)
			jacobian.col(system_->LZ.id) = full_jacobian.col(system_->LZ.id);
	} else {
		printf(YELLOW "Warning: this is a fixed-base robot\n" COLOR_RESET);
		jacobian = Eigen::MatrixXd::Zero(0,0);
	}
}


void WholeBodyKinematics::getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
											   const Eigen::MatrixXd& full_jacobian)
{
	enum rbd::TypeOfSystem type_of_system = system_->getTypeOfDynamicSystem();
	if (type_of_system == rbd::FloatingBase || type_of_system == rbd::ConstrainedFloatingBase)
		jacobian = full_jacobian.rightCols(system_->getJointDOF());
	else if (type_of_system == rbd::VirtualFloatingBase)
		jacobian = full_jacobian.rightCols(system_->getJointDOF());
	else
		jacobian = full_jacobian;
}


void WholeBodyKinematics::computeVelocity(rbd::BodyVector& op_vel,
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

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, system_);
			Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(base_vel, joint_vel, system_);

			// Computing the point velocity
			rbd::Vector6d point_vel = rbd::computePointVelocity(robot_model_, q, q_dot, body_id,
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


void WholeBodyKinematics::computeAcceleration(rbd::BodyVector& op_acc,
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

			Eigen::VectorXd q = rbd::toGeneralizedJointState(base_pos, joint_pos, system_);
			Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(base_vel, joint_vel, system_);
			Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(base_acc, joint_acc, system_);

			// Computing the point acceleration
			rbd::Vector6d point_acc = rbd::computePointAcceleration(robot_model_, q, q_dot, q_ddot,
					body_id, Eigen::Vector3d::Zero(), true);
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


void WholeBodyKinematics::computeJdotQdot(rbd::BodyVector& jacd_qd,
										  const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  const rbd::BodySelector& body_set,
										  enum rbd::Component component)
{
	// Getting the floating-base dof
	unsigned int num_joints = system_->getJointDOF();

	rbd::BodyVector op_vel, op_acc;
	computeAcceleration(op_acc, base_pos, joint_pos,
						base_vel, joint_vel,
						rbd::Vector6d::Zero(), Eigen::VectorXd::Zero(num_joints),
						body_set, component);

	// Resizing the acceleration contribution vector
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

	Eigen::VectorXd body_jacd_qd(num_vars);

	for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			switch (component) {
			case rbd::Linear: {
				// Computing the point velocity and its angular and linear components
				rbd::Vector6d point_vel = op_vel[body_name];
				Eigen::Vector3d ang_vel, lin_vel;
				ang_vel = rbd::angularPart(point_vel);
				lin_vel = rbd::linearPart(point_vel);

				// Computing the JdQd for current point
				body_jacd_qd.segment<3>(0) = op_acc[body_name] + ang_vel.cross(lin_vel);
				break;
			} case rbd::Angular: {
				// Computing the JdQd for current point
				body_jacd_qd.segment<3>(0) = op_acc[body_name];
				break;
			} case rbd::Full: {
				// Computing the point velocity and its angular and linear components
				rbd::Vector6d point_vel = op_vel[body_name];
				Eigen::Vector3d ang_vel, lin_vel;
				ang_vel = rbd::angularPart(point_vel);
				lin_vel = rbd::linearPart(point_vel);

				// Computing the JdQd for current point
				rbd::Vector6d point_acc = op_acc[body_name];
				body_jacd_qd.segment<3>(rbd::AX) = rbd::angularPart(point_acc);
				body_jacd_qd.segment<3>(rbd::LX) = rbd::linearPart(point_acc) + ang_vel.cross(lin_vel);
				break;}
			}

			jacd_qd[body_name] = body_jacd_qd;
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
