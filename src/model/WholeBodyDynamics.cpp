#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

WholeBodyDynamics::WholeBodyDynamics() : kin_model_(NULL), initialized_kinematics_(false)
{

}


WholeBodyDynamics::~WholeBodyDynamics()
{

}


void WholeBodyDynamics::setKinematicModel(WholeBodyKinematics* kinematics)
{
	kin_model_ = kinematics;

	initialized_kinematics_ = true;
}


void WholeBodyDynamics::opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																	const iit::rbd::Vector6D& base_pos,
																	const Eigen::VectorXd& joint_pos,
																	const iit::rbd::Vector6D& base_vel,
																	const Eigen::VectorXd& joint_vel,
																	enum Component component)
{
	if (!initialized_kinematics_)
		printf(RED "The kinematics model must be initialized " COLOR_RESET);

	// Computing the acceleration contribution from joint velocity for all end-effectors
	EndEffectorSelector effector_set;
	for (EndEffectorID::iterator effector_iter = kin_model_->getEndEffectorList().begin();
			effector_iter != kin_model_->getEndEffectorList().end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		effector_set[effector_name] = true;
	}

	opAccelerationContributionFromJointVelocity(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, effector_set, component);
}


void WholeBodyDynamics::opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																	const iit::rbd::Vector6D& base_pos,
																	const Eigen::VectorXd& joint_pos,
																	const iit::rbd::Vector6D& base_vel,
																	const Eigen::VectorXd& joint_vel,
																	EndEffectorSelector effector_set,
																	enum Component component) // TODO Compute for other cases (Angular and full)
{
	if (!initialized_kinematics_)
		printf(RED "The kinematics model must be initialized " COLOR_RESET);

	// Computing the number of active end-effectors
	int num_effector_set = 0;
	for (EndEffectorID::iterator effector_iter = kin_model_->getEndEffectorList().begin();
			effector_iter != kin_model_->getEndEffectorList().end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			++num_effector_set;
		}
	}
	jacd_qd.resize(3 * num_effector_set);
	jacd_qd.setZero();

	// Updating the dynamic and kinematic information
	propagateWholeBodyInverseDynamics(base_pos, joint_pos, base_vel, joint_vel,
									  iit::rbd::Vector6D::Zero(), Eigen::VectorXd::Zero(joint_pos.size()));
	kin_model_->updateState(base_pos, joint_pos);
	updateState(base_pos, joint_pos);

	// Computing the acceleration contribution from joint velocity, i.e. J_d*q_d
	iit::rbd::VelocityVector effector_vel;
	iit::rbd::VelocityVector effector_acc;
	for (EndEffectorID::iterator effector_iter = kin_model_->getEndEffectorList().begin();
			effector_iter != kin_model_->getEndEffectorList().end();
			effector_iter++)
	{
		int effector_counter = 0;
		std::string effector_name = effector_iter->second;
		if ((effector_set.find(effector_name)->second) && (effector_set.count(effector_name) > 0)) {
			effector_vel = closest_link_motion_tf_.find(effector_name)->second *
					closest_link_velocity_.find(effector_name)->second;
			effector_acc = closest_link_motion_tf_.find(effector_name)->second *
								closest_link_acceleration_.find(effector_name)->second;

			effector_acc.segment(iit::rbd::LX, 3) = iit::rbd::linearPart(effector_acc) +
					iit::rbd::angularPart(effector_vel).cross(iit::rbd::linearPart(effector_vel));

			Eigen::Matrix4d homogeneous_tf = kin_model_->getHomogeneousTransform(effector_name);
			jacd_qd.segment(effector_counter * 3, 3) = kin_model_->getBaseRotationMatrix().transpose()
					* iit::rbd::Utils::rotationMx(homogeneous_tf) * iit::rbd::linearPart(effector_acc);

			++effector_counter;
		}
	}
}

} //@namespace model
} //@namespace dwl
