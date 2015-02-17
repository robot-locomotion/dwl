#include <robot/HyLWholeBodyKinematics.h>


namespace dwl
{

namespace robot
{

HyLWholeBodyKinematics::HyLWholeBodyKinematics()
{

}


HyLWholeBodyKinematics::~HyLWholeBodyKinematics()
{

}


void HyLWholeBodyKinematics::init()
{
	// Defining the end-effector ids
	effector_id_[0] = "LF_foot";
	effector_id_[1] = "RF_foot";
	effector_id_[2] = "LH_foot";
//	effector_id_[3] = "RH_foot";

	// Defining the jacobians of the end-effectors
	current_jacs_["LF_foot"] = jacs_.fr_trunk_J_LF_foot;
	current_jacs_["RF_foot"] = jacs_.fr_trunk_J_RF_foot;
	current_jacs_["LH_foot"] = jacs_.fr_trunk_J_LH_foot;
	current_jacs_["RH_foot"] = jacs_.fr_trunk_J_RH_foot;

	// Computing the number of joints given the jacobians
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		Eigen::MatrixXd jac = current_jacs_.find(effector_name)->second;
		num_joints_ += jac.cols();
	}

	floating_base_rot_ = Eigen::Matrix3d::Identity();
}


void HyLWholeBodyKinematics::updateJointState(Eigen::VectorXd position, Eigen::VectorXd velocity)
{
	jacs_.fr_trunk_J_LF_foot(position);
	jacs_.fr_trunk_J_RF_foot(position);
	jacs_.fr_trunk_J_LH_foot(position);
	jacs_.fr_trunk_J_RH_foot(position);
	current_jacs_["LF_foot"] = jacs_.fr_trunk_J_LF_foot;
	current_jacs_["RF_foot"] = jacs_.fr_trunk_J_RF_foot;
	current_jacs_["LH_foot"] = jacs_.fr_trunk_J_LH_foot;
	current_jacs_["RH_foot"] = jacs_.fr_trunk_J_RH_foot;


	tf_.fr_trunk_X_LF_foot(position);
	tf_.fr_trunk_X_RF_foot(position);
	tf_.fr_trunk_X_LH_foot(position);
	tf_.fr_trunk_X_RH_foot(position);
	current_effector_pos_["LF_foot"] = iit::rbd::Utils::positionVector(tf_.fr_trunk_X_LF_foot);
	current_effector_pos_["RF_foot"] = iit::rbd::Utils::positionVector(tf_.fr_trunk_X_RF_foot);
	current_effector_pos_["LH_foot"] = iit::rbd::Utils::positionVector(tf_.fr_trunk_X_LH_foot);
	current_effector_pos_["RH_foot"] = iit::rbd::Utils::positionVector(tf_.fr_trunk_X_RH_foot);
}

} //@namespace robot
} //@namespace dwl
