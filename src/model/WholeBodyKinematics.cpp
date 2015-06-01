#include <model/WholeBodyKinematics.h>


namespace dwl
{

namespace model
{

WholeBodyKinematics::WholeBodyKinematics() : num_joints_(0)
{

}


WholeBodyKinematics::~WholeBodyKinematics()
{

}


void WholeBodyKinematics::computeEffectorFK(Eigen::VectorXd& op_pos,
											const Eigen::VectorXd& joint_pos,
											enum Component component)
{
	// Computing the forward kinematics for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

	computeEffectorFK(op_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeEffectorFK(Eigen::VectorXd& op_pos,
											const Eigen::VectorXd& joint_pos,
											EndEffectorSelector effector_set,
											enum Component component)
{
	// Updating the states
	updateState(iit::rbd::Vector6D::Zero(), joint_pos);

	// Computing the number of active end-effectors
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

	switch (component) {
	case Linear:
		op_pos.resize(3 * num_effector_set);
		break;
	case Angular:
		op_pos.resize(3 * num_effector_set);
		break;
	case Full:
		op_pos.resize(6 * num_effector_set);
		break;
	}
	op_pos.setZero();

	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			Eigen::Matrix4d homogeneous_tf = homogeneous_tf_.find(effector_name)->second;

			Orientation orientation(iit::rbd::Utils::rotationMx(homogeneous_tf));
			double r,p,y;
			orientation.getRPY(r,p,y);

			switch (component) {
			case Linear:
				op_pos = iit::rbd::Utils::positionVector(homogeneous_tf);
				break;
			case Angular:
				op_pos << r, p, y;
				break;
			case Full:
				op_pos << iit::rbd::Utils::positionVector(homogeneous_tf), r, p, y;
				break;
			}
		}
	}
}


void WholeBodyKinematics::computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
												   const iit::rbd::Vector6D& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

	computeWholeBodyJacobian(jacobian, base_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
												   const iit::rbd::Vector6D& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   EndEffectorSelector effector_set,
												   enum Component component)
{
	// Resizing the jacobian matrix
	int num_vars;
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
	default:
		num_vars = 6;
	}

	// Computing the number of active end-effectors
	int num_effector_set = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0)
			++num_effector_set;
	}
	jacobian.resize(num_vars * num_effector_set, 6 + num_joints_);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	Eigen::MatrixXd floating_base_jacobian, fixed_base_jacobian;
	computeFreeBaseJacobian(floating_base_jacobian, base_pos, joint_pos, effector_set, component);
	computeFixedBaseJacobian(fixed_base_jacobian, joint_pos, effector_set, component);

	jacobian << floating_base_jacobian, fixed_base_jacobian;

//	std::cout << "####################### WHOLE-BODY JACOBIAN #######################" << std::endl;
//	std::cout << jacobian << std::endl;
}


void WholeBodyKinematics::computeFreeBaseJacobian(Eigen::MatrixXd& jacobian,
												  const iit::rbd::Vector6D& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

	computeFreeBaseJacobian(jacobian, base_pos, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeFreeBaseJacobian(Eigen::MatrixXd& jacobian,
												  const iit::rbd::Vector6D& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  EndEffectorSelector effector_set,
												  enum Component component)
{
	// Updating the state
	updateState(base_pos, joint_pos);

	// Resizing the jacobian matrix
	int num_vars;
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
	default:
		num_vars = 6;
	}

	// Computing the number of active end-effectors
	int num_effector_set = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0)
			++num_effector_set;
		else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, effector_name.c_str());
	}
	jacobian.resize(num_vars * num_effector_set, 6);
	jacobian.setZero();

	// Computing the current position of each end-effector
	Eigen::VectorXd effector_positions;
	computeEffectorFK(effector_positions, joint_pos, effector_set, Linear);

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0) {
			int init_row = effector_counter * num_vars;
			Eigen::Vector3d foot_pos = effector_positions.segment(effector_counter * 3, 3);
			switch(component) {
			case Linear:
				jacobian.block(init_row, rbd::AX, 3, 3) = -floating_base_rot_.transpose() *
						math::skewSymmentricMatrixFrom3DVector(foot_pos);
				jacobian.block(init_row, rbd::LX, 3, 3) = floating_base_rot_.transpose();// TODO compute rotation matrix from base_pos
				break;
			case Angular:
				jacobian.block(init_row, rbd::AX, 3, 3) = floating_base_rot_.transpose();
				break;
			case Full:
				jacobian.block(init_row, rbd::AX, 3, 3) = floating_base_rot_.transpose();
				jacobian.block(init_row + 3, rbd::AX, 3, 3) = -floating_base_rot_.transpose() *
						math::skewSymmentricMatrixFrom3DVector(foot_pos);
				jacobian.block(init_row + 3, rbd::LX, 3, 3) = floating_base_rot_.transpose();
				break;
			}
			++effector_counter;
//			std::cout << "---------------------------------------------------" << std::endl;
//			std::cout << current_jacs_.find(effector_name)->second << std::endl;
		}
	}

//	std::cout << "###################### FLOATING-BASE JACOBIAN ########################" << std::endl;
//	std::cout << jacobian << std::endl;
}


void WholeBodyKinematics::computeFixedBaseJacobian(Eigen::MatrixXd& jacobian,
												   const Eigen::VectorXd& joint_pos,
												   enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

	computeFixedBaseJacobian(jacobian, joint_pos, effector_set, component);
}


void WholeBodyKinematics::computeFixedBaseJacobian(Eigen::MatrixXd& jacobian,
												   const Eigen::VectorXd& joint_pos,
												   EndEffectorSelector effector_set,
												   enum Component component)
{
	// Updating the state of the kinematic model
	updateState(iit::rbd::Vector6D::Zero(), joint_pos);

	// Resizing the jacobian matrix
	int num_vars;
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
	default:
		num_vars = 6;
	}

	// Computing the number of active end-effectors
	int num_effector_set = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		if (effector_id_.count(effector_name) > 0)
			++num_effector_set;
		else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, effector_name.c_str());
	}
	jacobian.resize(num_vars * num_effector_set, num_joints_);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		int effector_id = effector_id_.find(effector_name)->second;
		if (effector_id_.count(effector_name) > 0) {
			Eigen::MatrixXd jac = jacobians_.find(effector_name)->second;

			int num_jnts = jac.cols();
			int init_row = effector_counter * num_vars;
			int init_col = effector_id * num_jnts;
			switch (component) {
			case Linear:
				jacobian.block(init_row, init_col, num_vars, num_jnts) =
						floating_base_rot_.transpose() * jac.block(rbd::LX, 0, num_vars, num_jnts);
				break;
			case Angular:
				jacobian.block(init_row, init_col, num_vars, num_jnts) = jac.block(rbd::AX, 0, num_vars, num_jnts);
				break;
			case Full:
				jacobian.block(init_row + rbd::AX, init_col, num_vars / 2, num_jnts) = jac.block(rbd::AX, 0, num_vars / 2, num_jnts);
				jacobian.block(init_row + rbd::LX, init_col, num_vars / 2, num_jnts) = floating_base_rot_.transpose() *
						jac.block(rbd::LX, 0, num_vars / 2, num_jnts);
				break;
			}
			++effector_counter;
//			std::cout << "---------------------------------------------------" << std::endl;
//			std::cout << current_jacs_.find(effector_name)->second << std::endl;
		}
	}
//	std::cout << "###################### END-EFFECTOR JACOBIAN ########################" << std::endl;
//	std::cout << jacobian << std::endl;
}


void WholeBodyKinematics::opVelocityFromJointSpace(Eigen::VectorXd& op_vel,
												   const iit::rbd::Vector6D& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   const iit::rbd::Vector6D& base_vel,
												   const Eigen::VectorXd& joint_vel,
												   enum Component component)
{
	// Computing the operational velocities for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

	opVelocityFromJointSpace(op_vel, base_pos, joint_pos, base_vel, joint_vel, effector_set, component);
}


void WholeBodyKinematics::opVelocityFromJointSpace(Eigen::VectorXd& op_vel,
												   const iit::rbd::Vector6D& base_pos,
												   const Eigen::VectorXd& joint_pos,
												   const iit::rbd::Vector6D& base_vel,
												   const Eigen::VectorXd& joint_vel,
												   EndEffectorSelector effector_set,
												   enum Component component)
{
	// Computing the effector Jacobian
	Eigen::MatrixXd jacobian;
	computeWholeBodyJacobian(jacobian, base_pos, joint_pos, effector_set, component);

	// Computing the operation velocity
	Eigen::VectorXd q_vel = Eigen::VectorXd::Zero(6 + num_joints_);
	q_vel << base_vel, joint_vel;
	op_vel = jacobian * q_vel;
}


void WholeBodyKinematics::opAccelerationContributionFromJointAcceleration(Eigen::VectorXd& jac_qdd,
																		  const iit::rbd::Vector6D& base_pos,
																		  const Eigen::VectorXd& joint_pos,
																		  const iit::rbd::Vector6D& base_acc,
																		  const Eigen::VectorXd& joint_acc,
																		  enum Component component)
{
	// Computing the operational accelerations for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set[effector_name] = true;
	}

	opAccelerationContributionFromJointAcceleration(jac_qdd, base_pos, joint_pos, base_acc, joint_acc,
												  	effector_set, component);
}


void WholeBodyKinematics::opAccelerationContributionFromJointAcceleration(Eigen::VectorXd& jac_qdd,
																		  const iit::rbd::Vector6D& base_pos,
																		  const Eigen::VectorXd& joint_pos,
																		  const iit::rbd::Vector6D& base_acc,
																		  const Eigen::VectorXd& joint_acc,
																		  EndEffectorSelector effector_set,
																		  enum Component component)
{
	// Computing the effector Jacobian
	Eigen::MatrixXd jacobian;
	computeWholeBodyJacobian(jacobian, base_pos, joint_pos, effector_set, component);

	// Computing the operation acceleration
	Eigen::VectorXd q_acc = Eigen::VectorXd::Zero(6 + num_joints_);
	q_acc << base_acc, joint_acc;
	jac_qdd = jacobian * q_acc;
}


EndEffectorID& WholeBodyKinematics::getEndEffectorList()
{
	return effector_id_;
}

Eigen::Matrix3d WholeBodyKinematics::getBaseRotationMatrix()
{
	return floating_base_rot_;
}


Eigen::Matrix4d WholeBodyKinematics::getHomogeneousTransform(std::string effector_name)
{
	return homogeneous_tf_.find(effector_name)->second;
}

} //@namespace model
} //@namespace dwl
