#include <robot/HyLWholeBodyDynamics.h>


namespace dwl
{

namespace robot
{

HyLWholeBodyDynamics::HyLWholeBodyDynamics() : id_(inertia_, motion_tf_)
{

}


HyLWholeBodyDynamics::~HyLWholeBodyDynamics()
{

}


void HyLWholeBodyDynamics::init()
{
	// Defining the end-effector ids
	effector_id_[0] = "foot";

	// Defining the end-effector motion transform to the closest link
	closest_link_motion_tf_["foot"] = motion_tf_.fr_foot_X_fr_lowerleg;

	// Defining the velocity and acceleration to the closes link of the end-effector
	closest_link_velocity_["foot"] = id_.getVelocity_lowerleg();
	closest_link_acceleration_["foot"] = id_.getAcceleration_lowerleg();
}


void HyLWholeBodyDynamics::updateState(const iit::rbd::Vector6D& base_pos, const Eigen::VectorXd& joint_pos)
{
	// Computing the HyL state
	Eigen::Vector3d state;
	state << base_pos(iit::rbd::LZ), joint_pos;

	// Updating the end-effector motion transform to the closest link
	closest_link_motion_tf_["foot"] = motion_tf_.fr_foot_X_fr_lowerleg(state);

	// Updating the velocity and acceleration to the closes link of the end-effector
	closest_link_velocity_["foot"] = id_.getVelocity_lowerleg();
	closest_link_acceleration_["foot"] = id_.getAcceleration_lowerleg();
}


void HyLWholeBodyDynamics::computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
																		  const iit::rbd::Vector6D& base_pos,
																		  const iit::rbd::Vector6D& base_vel,
																		  const Eigen::VectorXd& joint_pos,
																		  const Eigen::VectorXd& joint_vel)
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

	computeJointVelocityContributionOfAcceleration(jacd_qd, effector_set, base_pos, base_vel, joint_pos, joint_vel);
}


void HyLWholeBodyDynamics::computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
																		  EndEffectorSelector effector_set,
																		  const iit::rbd::Vector6D& base_pos,
																		  const iit::rbd::Vector6D& base_vel,
																		  const Eigen::VectorXd& joint_pos,
																		  const Eigen::VectorXd& joint_vel)
{
	// Computing the HyL state
	Eigen::Vector3d state, state_d, state_dd;
	state << base_pos(iit::rbd::LZ), joint_pos.head(2);
	state_d << base_vel(iit::rbd::LZ), joint_vel.head(2);
	state_dd = Eigen::Vector3d::Zero();

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
	jacd_qd.resize(3 * num_effector_set);
	jacd_qd.setZero();

	// Updating the dynamic and kinematic information
	id_.setJointStatus(state);
	id_.propagateVelAcc(state_d, state_dd);
	kin_model_->updateState(base_pos, joint_pos);
	updateState(base_pos, joint_pos);

	// Computing the JointVelocityContributionOfAcceleration, i.e. J_dot*q_dot
	iit::rbd::VelocityVector effector_vel;
	iit::rbd::VelocityVector effector_acc;
	for (EndEffectorID::iterator effector_iter = effector_id_.begin();
			effector_iter != effector_id_.end();
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


void HyLWholeBodyDynamics::computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
		 	 	 	 	 	 	 	 	 	 	 	 	   const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_pos,
		 	 	 	 	 	 	 	 	 	 	 	 	   const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_acc,
		 	 	 	 	 	 	 	 	 	 	 	 	   const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& joint_vel,
		 	 	 	 	 	 	 	 	 	 	 	 	   const Eigen::VectorXd& joint_acc)
{
	// HyL model defines the slider as actuated joint, which is the floating-base. Therefore, the floating-base wrench,
	// velocity and acceleration is converted as joint variables to the ID algorithm
	Eigen::Vector3d tau, jnt_pos, jnt_vel, jnt_acc;
	jnt_pos << base_pos(iit::rbd::LZ), joint_pos;
	jnt_vel << base_vel(iit::rbd::LZ), joint_vel;
	jnt_acc << base_acc(iit::rbd::LZ), joint_acc;

	// Computing the inverse dynamics using the generated code of HyL
	id_.id(tau, jnt_pos, jnt_vel, jnt_acc);//const ExtForces& fext = zeroExtForces)

	// HyL only generate z force in the base
	base_wrench = iit::rbd::Vector6D::Zero();
	base_wrench(iit::rbd::LZ) = tau(0);

	// Joint forces according the ID model
	joint_forces = tau.tail(2);


//	std::cout << "Base wrench = " << base_wrench.transpose() << std::endl;
//	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;
}

} //@namespace robot
} //namespace dwl
