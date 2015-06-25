#include <model/RobCoGenWholeBodyKinematics.h>


namespace dwl
{

namespace model
{

RobCoGenWholeBodyKinematics::RobCoGenWholeBodyKinematics() : num_joints_(0)
{

}


RobCoGenWholeBodyKinematics::~RobCoGenWholeBodyKinematics()
{

}


void RobCoGenWholeBodyKinematics::computeForwardKinematics(Eigen::VectorXd& op_pos,
														   const Eigen::VectorXd& joint_pos,
														   rbd::BodySelector body_set,
														   enum rbd::Component component)
{
	// Updating the states
	updateState(iit::rbd::Vector6D::Zero(), joint_pos);

	// Computing the number of active end-effectors
	int num_body_set = 0;
	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			++num_body_set;
		} else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, body_name.c_str());
	}

	switch (component) {
	case rbd::Linear:
		op_pos.resize(3 * num_body_set);
		break;
	case rbd::Angular:
		op_pos.resize(3 * num_body_set);
		break;
	case rbd::Full:
		op_pos.resize(6 * num_body_set);
		break;
	}
	op_pos.setZero();

	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			Eigen::Matrix4d homogeneous_tf = homogeneous_tf_.find(body_name)->second;

			Eigen::Vector3d rpy = math::getRPY((Eigen::Matrix3d) iit::rbd::Utils::rotationMx(homogeneous_tf));

			switch (component) {
			case rbd::Linear:
				op_pos = iit::rbd::Utils::positionVector(homogeneous_tf);
				break;
			case rbd::Angular:
				op_pos << rpy;
				break;
			case rbd::Full:
				op_pos << iit::rbd::Utils::positionVector(homogeneous_tf), rpy;
				break;
			}
		}
	}
}


void RobCoGenWholeBodyKinematics::computeJacobian(Eigen::MatrixXd& jacobian,
												  const rbd::Vector6d& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  rbd::BodySelector body_set,
												  enum rbd::Component component)
{
	// Resizing the jacobian matrix
	int num_vars;
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
	default:
		num_vars = 6;
	}

	// Computing the number of active end-effectors
	int num_body_set = 0;
	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0)
			++num_body_set;
	}
	jacobian.resize(num_vars * num_body_set, 6 + num_joints_);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	Eigen::MatrixXd floating_base_jacobian, fixed_base_jacobian;
	computeFloatingBaseJacobian(floating_base_jacobian, base_pos, joint_pos, body_set, component);
	computeFixedBaseJacobian(fixed_base_jacobian, joint_pos, body_set, component);

	jacobian << floating_base_jacobian, fixed_base_jacobian;
}


void RobCoGenWholeBodyKinematics::computeFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
															  const rbd::Vector6d& base_pos,
															  const Eigen::VectorXd& joint_pos,
															  rbd::BodySelector body_set,
															  enum rbd::Component component)
{
	// Updating the state
	updateState(base_pos, joint_pos);

	// Resizing the jacobian matrix
	int num_vars;
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
	default:
		num_vars = 6;
	}

	// Computing the number of active end-effectors
	int num_body_set = 0;
	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0)
			++num_body_set;
		else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, body_name.c_str());
	}
	jacobian.resize(num_vars * num_body_set, 6);
	jacobian.setZero();

	// Computing the current position of each end-effector
	Eigen::VectorXd effector_positions;
	computeForwardKinematics(effector_positions, joint_pos, body_set, rbd::Linear);

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0) {
			int init_row = effector_counter * num_vars;
			Eigen::Vector3d foot_pos = effector_positions.segment(effector_counter * 3, 3);
			switch(component) {
			case rbd::Linear:
				jacobian.block(init_row, rbd::AX, 3, 3) = -floating_base_rot_.transpose() *
						math::skewSymmentricMatrixFrom3DVector(foot_pos);
				jacobian.block(init_row, rbd::LX, 3, 3) = floating_base_rot_.transpose();// TODO compute rotation matrix from base_pos
				break;
			case rbd::Angular:
				jacobian.block(init_row, rbd::AX, 3, 3) = floating_base_rot_.transpose();
				break;
			case rbd::Full:
				jacobian.block(init_row, rbd::AX, 3, 3) = floating_base_rot_.transpose();
				jacobian.block(init_row + 3, rbd::AX, 3, 3) = -floating_base_rot_.transpose() *
						math::skewSymmentricMatrixFrom3DVector(foot_pos);
				jacobian.block(init_row + 3, rbd::LX, 3, 3) = floating_base_rot_.transpose();
				break;
			}
			++effector_counter;
		}
	}
}


void RobCoGenWholeBodyKinematics::computeFixedBaseJacobian(Eigen::MatrixXd& jacobian,
														   const Eigen::VectorXd& joint_pos,
														   rbd::BodySelector body_set,
														   enum rbd::Component component)
{
	// Updating the state of the kinematic model
	updateState(iit::rbd::Vector6D::Zero(), joint_pos);

	// Resizing the jacobian matrix
	int num_vars;
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
	default:
		num_vars = 6;
	}

	// Computing the number of active end-effectors
	int num_body_set = 0;
	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		if (body_id_.count(body_name) > 0)
			++num_body_set;
		else
			printf(YELLOW "WARNING: The %s link is not an end-effector\n" COLOR_RESET, body_name.c_str());
	}
	jacobian.resize(num_vars * num_body_set, num_joints_);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (rbd::BodySelector::iterator body_iter = body_set.begin();
			body_iter != body_set.end();
			body_iter++)
	{
		std::string body_name = *body_iter;
		int effector_id = body_id_.find(body_name)->second;
		if (body_id_.count(body_name) > 0) {
			Eigen::MatrixXd jac = jacobians_.find(body_name)->second;

			int num_jnts = jac.cols();
			int init_row = effector_counter * num_vars;
			int init_col = effector_id * num_jnts;
			switch (component) {
			case rbd::Linear:
				jacobian.block(init_row, init_col, num_vars, num_jnts) =
						floating_base_rot_.transpose() * jac.block(rbd::LX, 0, num_vars, num_jnts);
				break;
			case rbd::Angular:
				jacobian.block(init_row, init_col, num_vars, num_jnts) = jac.block(rbd::AX, 0, num_vars, num_jnts);
				break;
			case rbd::Full:
				jacobian.block(init_row + rbd::AX, init_col, num_vars / 2, num_jnts) = jac.block(rbd::AX, 0, num_vars / 2, num_jnts);
				jacobian.block(init_row + rbd::LX, init_col, num_vars / 2, num_jnts) = floating_base_rot_.transpose() *
						jac.block(rbd::LX, 0, num_vars / 2, num_jnts);
				break;
			}
			++effector_counter;
		}
	}
}


void RobCoGenWholeBodyKinematics::opVelocityFromJointSpace(Eigen::VectorXd& op_vel,
														   const rbd::Vector6d& base_pos,
														   const Eigen::VectorXd& joint_pos,
														   const rbd::Vector6d& base_vel,
														   const Eigen::VectorXd& joint_vel,
														   rbd::BodySelector body_set,
														   enum rbd::Component component)
{
	// Computing the effector Jacobian
	Eigen::MatrixXd jacobian;
	computeJacobian(jacobian, base_pos, joint_pos, body_set, component);

	// Computing the operation velocity
	Eigen::VectorXd q_vel = Eigen::VectorXd::Zero(6 + num_joints_);
	q_vel << base_vel, joint_vel;
	op_vel = jacobian * q_vel;
}


void RobCoGenWholeBodyKinematics::opAccelerationContributionFromJointAcceleration(Eigen::VectorXd& jac_qdd,
																				  const rbd::Vector6d& base_pos,
																				  const Eigen::VectorXd& joint_pos,
																				  const rbd::Vector6d& base_acc,
																				  const Eigen::VectorXd& joint_acc,
																				  rbd::BodySelector body_set,
																				  enum rbd::Component component)
{
	// Computing the effector Jacobian
	Eigen::MatrixXd jacobian;
	computeJacobian(jacobian, base_pos, joint_pos, body_set, component);

	// Computing the operation acceleration
	Eigen::VectorXd q_acc = Eigen::VectorXd::Zero(6 + num_joints_);
	q_acc << base_acc, joint_acc;
	jac_qdd = jacobian * q_acc;
}


rbd::BodyID& RobCoGenWholeBodyKinematics::getBodyList()
{
	return body_id_;
}

Eigen::Matrix3d RobCoGenWholeBodyKinematics::getBaseRotationMatrix()
{
	return floating_base_rot_;
}


Eigen::Matrix4d RobCoGenWholeBodyKinematics::getHomogeneousTransform(std::string body_name)
{
	return homogeneous_tf_.find(body_name)->second;
}

} //@namespace model
} //@namespace dwl
