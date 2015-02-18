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


void WholeBodyKinematics::computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
												   enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		effector_set[effector_name] = true;
	}

	computeWholeBodyJacobian(jacobian, effector_set, component);
}


void WholeBodyKinematics::computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
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
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			++num_effector_set;
		}
	}
	jacobian.resize(num_vars * num_effector_set, 6 + num_joints_);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	Eigen::MatrixXd floating_base_jacobian, fixed_base_jacobian;
	computeBaseJacobian(floating_base_jacobian, effector_set, component);
	computeEffectorJacobian(fixed_base_jacobian, effector_set, component);

	jacobian << floating_base_jacobian, fixed_base_jacobian;

	std::cout << "####################### WHOLE-BODY JACOBIAN #######################" << std::endl;
	std::cout << jacobian << std::endl;
}


void WholeBodyKinematics::computeBaseJacobian(Eigen::MatrixXd& jacobian,
											  enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		effector_set[effector_name] = true;
	}

	computeBaseJacobian(jacobian, effector_set, component);
}


void WholeBodyKinematics::computeBaseJacobian(Eigen::MatrixXd& jacobian,
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
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			++num_effector_set;
		}
	}
	jacobian.resize(num_vars * num_effector_set, 6);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			int init_row = effector_counter * num_vars;
			Eigen::Vector3d foot_pos = current_effector_pos_.find(effector_name)->second;
			switch(component) {
			case Linear:
				jacobian.block(init_row, rbd::AX, 3, 3) = -floating_base_rot_.transpose() *
						math::skewSymmentricMatrixFrom3DVector(foot_pos);
				jacobian.block(init_row, rbd::LX, 3, 3) = floating_base_rot_.transpose();
				break;
			case Angular:
				jacobian.block(init_row, rbd::AX, 3, 3) = floating_base_rot_.transpose();
				break;
			case Full:
				jacobian.block(init_row, rbd::AX, 3, 3) = -floating_base_rot_.transpose() *
						math::skewSymmentricMatrixFrom3DVector(foot_pos);
				jacobian.block(init_row, rbd::LX, 3, 3) = floating_base_rot_.transpose();
				jacobian.block(init_row + 3, rbd::AX, 3, 3) = floating_base_rot_.transpose();
				break;
			}
			++effector_counter;
			std::cout << "---------------------------------------------------" << std::endl;
			std::cout << current_jacs_.find(effector_name)->second << std::endl;
		}
	}

	std::cout << "###################### FLOATING-BASE JACOBIAN ########################" << std::endl;
	std::cout << jacobian << std::endl;
}


void WholeBodyKinematics::computeEffectorJacobian(Eigen::MatrixXd& jacobian,
												  enum Component component)
{
	// Computing the jacobian for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		effector_set[effector_name] = true;
	}

	computeEffectorJacobian(jacobian, effector_set, component);
}


void WholeBodyKinematics::computeEffectorJacobian(Eigen::MatrixXd& jacobian,
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
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			++num_effector_set;
		}
	}
	jacobian.resize(num_vars * num_effector_set, num_joints_);
	jacobian.setZero();

	// Adding the jacobian only for the active end-effectors
	int effector_counter = 0;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		int effector_id = effector_iter->first;
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			Eigen::MatrixXd jac = current_jacs_.find(effector_name)->second;
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
			std::cout << "---------------------------------------------------" << std::endl;
			std::cout << current_jacs_.find(effector_name)->second << std::endl;
		}
	}
	std::cout << "###################### END-EFFECTOR JACOBIAN ########################" << std::endl;
	std::cout << jacobian << std::endl;
}

} //@namespace model
} //@namespace dwl
