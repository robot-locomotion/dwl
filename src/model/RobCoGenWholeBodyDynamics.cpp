#include <model/RobCoGenWholeBodyDynamics.h>


namespace dwl
{

namespace model
{

RobCoGenWholeBodyDynamics::RobCoGenWholeBodyDynamics() : kin_model_(NULL), initialized_kinematics_(false)
{

}


RobCoGenWholeBodyDynamics::~RobCoGenWholeBodyDynamics()
{

}


void RobCoGenWholeBodyDynamics::setKinematicModel(RobCoGenWholeBodyKinematics* kinematics)
{
	kin_model_ = kinematics;

	initialized_kinematics_ = true;
}


void RobCoGenWholeBodyDynamics::opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																			const rbd::Vector6d& base_pos,
																			const Eigen::VectorXd& joint_pos,
																			const rbd::Vector6d& base_vel,
																			const Eigen::VectorXd& joint_vel,
																			enum rbd::Component component)
{
	if (!initialized_kinematics_)
		printf(RED "The kinematics model must be initialized " COLOR_RESET);

	// Computing the acceleration contribution from joint velocity for all end-effectors
	rbd::EndEffectorSelector effector_set;
	for (rbd::EndEffectorID::iterator effector_iter = kin_model_->getEndEffectorList().begin();
			effector_iter != kin_model_->getEndEffectorList().end();
			effector_iter++)
	{
		std::string effector_name = effector_iter->first;
		effector_set.push_back(effector_name);
	}

	opAccelerationContributionFromJointVelocity(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, effector_set, component);
}


void RobCoGenWholeBodyDynamics::opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																			const rbd::Vector6d& base_pos,
																			const Eigen::VectorXd& joint_pos,
																			const rbd::Vector6d& base_vel,
																			const Eigen::VectorXd& joint_vel,
																			rbd::EndEffectorSelector effector_set,
																			enum rbd::Component component) // TODO Compute for other cases (Angular and full)
{
	if (!initialized_kinematics_)
		printf(RED "The kinematics model must be initialized " COLOR_RESET);

	// Computing the number of active end-effectors
	int num_effector_set = 0;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		std::string effector_name = *effector_iter;
		if (kin_model_->getEndEffectorList().count(effector_name) > 0)
			++num_effector_set;
	}
	jacd_qd.resize(3 * num_effector_set);
	jacd_qd.setZero();

	// Updating the dynamic and kinematic information
	propagateInverseDynamics(base_pos, joint_pos, base_vel, joint_vel,
							 rbd::Vector6d::Zero(), Eigen::VectorXd::Zero(joint_pos.size()));
	kin_model_->updateState(base_pos, joint_pos);
	updateState(base_pos, joint_pos);

	// Computing the acceleration contribution from joint velocity, i.e. J_d*q_d
	iit::rbd::VelocityVector effector_vel;
	iit::rbd::VelocityVector effector_acc;
	for (rbd::EndEffectorSelector::iterator effector_iter = effector_set.begin();
			effector_iter != effector_set.end();
			effector_iter++)
	{
		int effector_counter = 0;
		std::string effector_name = *effector_iter;
		if (kin_model_->getEndEffectorList().count(effector_name) > 0) {
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
