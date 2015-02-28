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
	effector_id_[0] = "foot";

	// Defining the jacobians of the end-effectors
	jacobians_["foot"] = jacs_.fr_trunk_J_fr_foot;

	// Defining the homogeneous transforms of the end-effectors
	homogeneous_tf_["foot"] = hom_tf_.fr_trunk_X_fr_foot;

	// Computing the number of joints given the jacobians
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		Eigen::MatrixXd jac = jacobians_.find(effector_name)->second;
		num_joints_ += jac.cols();
	}

	floating_base_rot_ = Eigen::Matrix3d::Identity();
}


void HyLWholeBodyKinematics::updateState(const iit::rbd::Vector6D& base_pos, const Eigen::VectorXd& joint_pos)
{
//	if (joint_pos.size() > 2)
//		printf("Error the joint position must be 2");

	// Computing the HyL state
	Eigen::Vector3d state;
	state << base_pos(iit::rbd::LZ), joint_pos;

	// Updating jacobians
	jacobians_["foot"] = jacs_.fr_trunk_J_fr_foot(state);

	// Updating homogeneous transforms
	homogeneous_tf_["foot"] = hom_tf_.fr_trunk_X_fr_foot(state);
}

} //@namespace robot
} //@namespace dwl
