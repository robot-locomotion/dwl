#include <model/WholeBodyKinematics.h>

#include <rbdl/rbdl_mathutils.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics() //: num_joints_(0)
{
	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);


	std::cout << "Degree of freedom overview:" << std::endl;
	std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot_model_);
	std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(robot_model_);


	std::cout << "Model Hierarchy:" << std::endl;
	std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(robot_model_);
}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::addEndEffector(std::string name)
{
	int effector_id = robot_model_.GetBodyId(name.c_str());
	effector_id_[name.c_str()] = effector_id;
}




void WholeBodyKinematics::computeWholeBodyFK(Eigen::VectorXd& op_pos,
											const Vector6d& base_pos,
											const Eigen::VectorXd& joint_pos,
											enum Component component,
											enum TypeOfOrientation type)
{
	// Computing the forward kinematics for all end-effectors
	EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeWholeBodyFK(op_pos, base_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeWholeBodyFK(Eigen::VectorXd& op_pos,
											const Vector6d& base_pos,
											const Eigen::VectorXd& joint_pos,
											EndEffectorSelector effector_set,
											enum Component component,
											enum TypeOfOrientation type)
{
	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	// Resizing the whole-body operation position vector
	int lin_vars = 0, ang_vars = 0;
	switch(component) {
	case Linear:
		ang_vars = 0;
		lin_vars = 3;
		break;
	case Angular:
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
	case Full:
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
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			unsigned int effector_id = effector_id_.find(effector_name)->second;
			int init_ang_col = effector_counter * ang_vars;
			int init_lin_col = effector_counter * lin_vars;

			Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);

			Eigen::Matrix3d rotation_mtx;
			switch (component) {
			case Linear:
				op_pos.segment(init_lin_col, lin_vars) =
						CalcBodyToBaseCoordinates(robot_model_, q, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			case Angular:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, effector_id, false);
				switch (type) {
					case RollPitchYaw:
						op_pos.segment(init_ang_col, ang_vars) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						op_pos.segment(init_ang_col, ang_vars) = math::getQuaternion(rotation_mtx).vec();
						break;
					case RotationMatrix:
						break;
				}
				break;
			case Full:
				rotation_mtx = RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, q, effector_id, false);
				switch (type) {
					case RollPitchYaw:
						op_pos.segment(init_ang_col, ang_vars) = math::getRPY(rotation_mtx);
						break;
					case Quaternion:
						op_pos.segment(init_ang_col, ang_vars) = math::getQuaternion(rotation_mtx).vec();
						break;
					case RotationMatrix:
						break;
				}

				// Computing the linear component
				op_pos.segment(init_ang_col + init_lin_col + ang_vars, lin_vars) =
						CalcBodyToBaseCoordinates(robot_model_, q, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			}
		}
	}
}


void WholeBodyKinematics::computeWholeBodyIK(Vector6d& base_pos,
								   Eigen::VectorXd& joint_pos,
								   const Vector6d& base_pos_init,
								   const Eigen::VectorXd& joint_pos_init,
								   const Eigen::VectorXd& op_pos)
{
	// Computing the inverse kinematics for all end-effectors
	EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeWholeBodyIK(base_pos, joint_pos, base_pos_init, joint_pos_init, op_pos, effector_set);
}


void WholeBodyKinematics::computeWholeBodyIK(Vector6d& base_pos,
								   Eigen::VectorXd& joint_pos,
								   const Vector6d& base_pos_init,
								   const Eigen::VectorXd& joint_pos_init,
								   const Eigen::VectorXd& op_pos,
								   EndEffectorSelector effector_set)
{
	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	assert(op_pos.size() == (num_effector_set*3));

	int effector_counter = 0;
	std::vector<unsigned int> body_id;
	std::vector<RigidBodyDynamics::Math::Vector3d> body_point;
	std::vector<RigidBodyDynamics::Math::Vector3d> target_pos;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			body_id.push_back(effector_id_.find(effector_name)->second);
			body_point.push_back(RigidBodyDynamics::Math::Vector3d::Zero());
			target_pos.push_back((Eigen::Vector3d) op_pos.segment(effector_counter*3,3));

			++effector_counter;
		}
	}


	Eigen::VectorXd q_init = toGeneralizedJointState(base_pos_init, joint_pos_init);
	Eigen::VectorXd q_res;
	RigidBodyDynamics::InverseKinematics(robot_model_, q_init, body_id, body_point, target_pos, q_res);

	fromGeneralizedJointState(base_pos, joint_pos, q_res);
}


void WholeBodyKinematics::computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
												   const Vector6d& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeWholeBodyJacobian(jacobian, base_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
												   const Vector6d& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   EndEffectorSelector effector_set,
												   enum Component component)
{
	// Resizing the jacobian matrix
	int num_vars = 0;
	switch (component) {
	case Linear:
		num_vars = 3;
		break;
	case Angular:
		num_vars = 3;
		break;
	case Full:
		num_vars = 6;
		break;
	}

	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	jacobian.resize(num_vars * num_effector_set, robot_model_.dof_count);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		int init_row = effector_counter * num_vars;

		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			int effector_id = effector_id_.find(effector_name)->second;

			Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);

			Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(6, robot_model_.dof_count));
			computePointJacobian(q, effector_id, Eigen::VectorXd::Zero(robot_model_.dof_count), jac, true);

			switch(component) {
			case Linear:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac.block(3,0,6,robot_model_.dof_count);
				break;
			case Angular:
				jacobian.block(init_row, 0, num_vars, robot_model_.dof_count) = jac.block(0,0,3,robot_model_.dof_count);
				break;
			case Full:
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
	if (isFloatingBaseRobot())
		jacobian = full_jacobian.block<6,6>(0,0);
	else {
		printf(YELLOW "Warning: this is a fixed-base robot\n" COLOR_RESET);
		jacobian = Eigen::MatrixXd::Zero(6,6);
	}
}


void WholeBodyKinematics::getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
												   const Eigen::MatrixXd& full_jacobian)
{
	if (isFloatingBaseRobot())
		jacobian = full_jacobian.rightCols(robot_model_.dof_count - 6);
	else
		jacobian = full_jacobian;
}


void WholeBodyKinematics::computeWholeBodyVelocity(Eigen::VectorXd& op_vel,
		  const Vector6d& base_pos,
		  const Eigen::VectorXd& joint_pos,
		  const Vector6d& base_vel,
		  const Eigen::VectorXd& joint_vel,
		  enum Component component)
{
	// Computing the velocity for all end-effectors
	EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeWholeBodyVelocity(op_vel, base_pos, joint_pos, base_vel, joint_vel, effector_set, component);
}


void WholeBodyKinematics::computeWholeBodyVelocity(Eigen::VectorXd& op_vel,
		  const Vector6d& base_pos,
		  const Eigen::VectorXd& joint_pos,
		  const Vector6d& base_vel,
		  const Eigen::VectorXd& joint_vel,
		  EndEffectorSelector effector_set,
		  enum Component component)
{
	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	// Resizing the velocity vector
	int num_vars = 0;
	switch (component) {
	case Linear:
		num_vars = 3;
		break;
	case Angular:
		num_vars = 3;
		break;
	case Full:
		num_vars = 6;
		break;
	}
	op_vel.resize(num_effector_set * num_vars);

	// Adding the velocity only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			int effector_id = effector_id_.find(effector_name)->second;
			int init_col = effector_counter * num_vars;

			Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
			Eigen::VectorXd q_dot = toGeneralizedJointState(base_vel, joint_vel);

			switch (component) {
			case Linear:
				op_vel.segment(init_col, num_vars) =
						computePointVelocity(q, q_dot, effector_id, Eigen::Vector3d::Zero(), true).tail(3);
				break;
			case Angular:
				op_vel.segment(init_col, num_vars) =
						computePointVelocity(q, q_dot, effector_id, Eigen::Vector3d::Zero(), true).head(3);
				break;
			case Full:
				op_vel.segment(init_col, num_vars) =
						computePointVelocity(q, q_dot, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			++effector_counter;
		}
	}
}


void WholeBodyKinematics::computeWholeBodyAcceleration(Eigen::VectorXd& op_acc,
								  const Vector6d& base_pos,
								  const Eigen::VectorXd& joint_pos,
								  const Vector6d& base_vel,
								  const Eigen::VectorXd& joint_vel,
								  const Vector6d& base_acc,
								  const Eigen::VectorXd& joint_acc,
								  enum Component component)
{
	// Computing the acceleration for all end-effectors
	EndEffectorSelector effector_set;
	activeAllEndEffector(effector_set);

	computeWholeBodyAcceleration(op_acc, base_pos, joint_pos, base_vel, joint_vel,
								 base_acc, joint_acc, effector_set, component);
}


void WholeBodyKinematics::computeWholeBodyAcceleration(Eigen::VectorXd& op_acc,
								  const Vector6d& base_pos,
								  const Eigen::VectorXd& joint_pos,
								  const Vector6d& base_vel,
								  const Eigen::VectorXd& joint_vel,
								  const Vector6d& base_acc,
								  const Eigen::VectorXd& joint_acc,
								  EndEffectorSelector effector_set,
								  enum Component component)
{
	// Computing the number of active end-effectors
	int num_effector_set = getNumberOfActiveEndEffectors(effector_set);

	// Resizing the velocity vector
	int num_vars = 0;
	switch (component) {
	case Linear:
		num_vars = 3;
		break;
	case Angular:
		num_vars = 3;
		break;
	case Full:
		num_vars = 6;
		break;
	}
	op_acc.resize(num_effector_set * num_vars);

	// Adding the velocity only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			int effector_id = effector_id_.find(effector_name)->second;
			int init_col = effector_counter * num_vars;

			Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
			Eigen::VectorXd q_dot = toGeneralizedJointState(base_vel, joint_vel);
			Eigen::VectorXd q_ddot = toGeneralizedJointState(base_acc, joint_acc);

			switch (component) {
			case Linear:
				op_acc.segment(init_col, num_vars) =
						computePointAcceleration(q, q_dot, q_ddot, effector_id, Eigen::Vector3d::Zero(), true).tail(3);
				break;
			case Angular:
				op_acc.segment(init_col, num_vars) =
						computePointAcceleration(q, q_dot, q_ddot, effector_id, Eigen::Vector3d::Zero(), true).head(3);
				break;
			case Full:
				op_acc.segment(init_col, num_vars) =
						computePointAcceleration(q, q_dot, q_ddot, effector_id, Eigen::Vector3d::Zero(), true);
				break;
			}

			++effector_counter;
		}
	}
}



/*
EndEffectorID& WholeBodyKinematics::getEndEffectorList()
{
	return effector_id_;
}
*/


bool WholeBodyKinematics::isFloatingBaseRobot()
{
	bool is_floating_base = false;
	int i = 1;
	while (robot_model_.mBodies[i].mIsVirtual) {
		i = robot_model_.mu[i][0];
		if (i == 6)
			is_floating_base = true;
	}

	return is_floating_base;
}


Eigen::VectorXd WholeBodyKinematics::toGeneralizedJointState(const Vector6d& base_state,
																	  const Eigen::VectorXd& joint_state)
{
	Eigen::VectorXd q(robot_model_.dof_count);
	if (isFloatingBaseRobot())
		q << base_state, joint_state;
	else
		q = joint_state;

	return q;
}


void WholeBodyKinematics::fromGeneralizedJointState(Vector6d& base_state,
										   Eigen::VectorXd& joint_state,
										   const Eigen::VectorXd& generalized_state)
{
	if (isFloatingBaseRobot()) {
		base_state = generalized_state.head<6>();
		joint_state = generalized_state.tail(robot_model_.dof_count - 6);
	} else {
		base_state = Vector6d::Zero();
		joint_state = generalized_state;
	}
}


int WholeBodyKinematics::getNumberOfActiveEndEffectors(EndEffectorSelector effector_set)
{
	int num_effector_set = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			++num_effector_set;
		} else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, effector_name.c_str());
	}

	return num_effector_set;
}


void WholeBodyKinematics::activeAllEndEffector(EndEffectorSelector& effector_set)
{
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

}


void WholeBodyKinematics::computePointJacobian(const Eigen::VectorXd& q,
		  				     	 	 	 	 	 	 	unsigned int body_id,
		  				     	 	 	 	 	 	 	const Eigen::Vector3d& point_position,
		  				     	 	 	 	 	 	 	Eigen::MatrixXd& jacobian,
		  				     	 	 	 	 	 	 	bool update_kinematics)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;
	LOG << "-------- " << __func__ << " --------" << std::endl;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom(robot_model_, &q, NULL, NULL);
	}

	SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates(robot_model_, q, body_id, point_position, false));

	assert (G.rows() == 6 && G.cols() == robot_model_.qdot_size );

	unsigned int reference_body_id = body_id;

	if (robot_model_.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - robot_model_.fixed_body_discriminator;
		reference_body_id = robot_model_.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.
	while (j != 0) {
		unsigned int q_index = robot_model_.mJoints[j].q_index;

		if (robot_model_.mJoints[j].mDoFCount == 3) {
			jacobian.block(0, q_index, 6, 3) = ((point_trans * robot_model_.X_base[j].inverse()).toMatrix() * robot_model_.multdof3_S[j]);
		} else {
			jacobian.block(0,q_index, 6, 1) = point_trans.apply(robot_model_.X_base[j].inverse().apply(robot_model_.S[j]));
		}

		j = robot_model_.lambda[j];
	}
}


Vector6d WholeBodyKinematics::computePointVelocity(const Eigen::VectorXd& q,
		const Eigen::VectorXd& q_dot,
		unsigned int body_id,
		const Eigen::Vector3d point_position,
		bool update_kinematics)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	LOG << "-------- " << __func__ << " --------" << std::endl;
	assert (robot_model_.IsBodyId(body_id));
	assert (robot_model_.q_size == q.size());
	assert (robot_model_.qdot_size == q_dot.size());

	// Reset the velocity of the root body
	robot_model_.v[0].setZero();

	// update the Kinematics with zero acceleration
	if (update_kinematics) {
		UpdateKinematicsCustom(robot_model_, &q, &q_dot, NULL);
	}

	unsigned int reference_body_id = body_id;
	Vector3d reference_point = point_position;

	if (robot_model_.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - robot_model_.fixed_body_discriminator;
		reference_body_id = robot_model_.mFixedBodies[fbody_id].mMovableParent;
		Vector3d base_coords = CalcBodyToBaseCoordinates(robot_model_, q, body_id, point_position, false);
		reference_point = CalcBaseToBodyCoordinates(robot_model_, q, reference_body_id, base_coords, false);
	}

	SpatialVector point_spatial_velocity =
			SpatialTransform(CalcBodyWorldOrientation(robot_model_, q, reference_body_id, false).transpose(),
					reference_point).apply(robot_model_.v[reference_body_id]);

	return point_spatial_velocity;
}


Vector6d WholeBodyKinematics::computePointAcceleration(const Eigen::VectorXd& q,
		const Eigen::VectorXd& q_dot,
		const Eigen::VectorXd& q_ddot,
		unsigned int body_id,
		const Eigen::Vector3d point_position,
		bool update_kinematics)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	LOG << "-------- " << __func__ << " --------" << std::endl;

	// Reset the velocity of the root body
	robot_model_.v[0].setZero();
	robot_model_.a[0].setZero();

	if (update_kinematics)
		UpdateKinematics(robot_model_, q, q_dot, q_ddot);

	LOG << std::endl;

	unsigned int reference_body_id = body_id;
	Vector3d reference_point = point_position;

	if (robot_model_.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - robot_model_.fixed_body_discriminator;
		reference_body_id = robot_model_.mFixedBodies[fbody_id].mMovableParent;
		Vector3d base_coords = CalcBodyToBaseCoordinates(robot_model_, q, body_id, point_position, false);
		reference_point = CalcBaseToBodyCoordinates(robot_model_, q, reference_body_id, base_coords, false);
	}

	SpatialTransform p_X_i (CalcBodyWorldOrientation(robot_model_, q, reference_body_id, false).transpose(), reference_point);

	SpatialVector p_v_i = p_X_i.apply(robot_model_.v[reference_body_id]);
	Vector3d a_dash = Vector3d (p_v_i[0], p_v_i[1], p_v_i[2]).cross(Vector3d (p_v_i[3], p_v_i[4], p_v_i[5]));
	SpatialVector p_a_i = p_X_i.apply(robot_model_.a[reference_body_id]);

	Vector6d point_spatial_acceleration;
	point_spatial_acceleration << p_a_i[0], p_a_i[1], p_a_i[2],
								  p_a_i[3] + a_dash[0], p_a_i[4] + a_dash[1], p_a_i[5] + a_dash[2];

	return point_spatial_acceleration;
}

} //@namespace model
} //@namespace dwl
