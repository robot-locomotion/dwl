#include <robot/HyLWholeBodyDynamics.h>


namespace dwl
{

namespace robot
{

HyLWholeBodyDynamics::HyLWholeBodyDynamics() : id_(inertia_, motion_tf_)
{
	// Defining the end-effector motion transform to the closest link
	closest_link_motion_tf_["foot"] = motion_tf_.fr_foot_X_fr_lowerleg;

	// Defining the velocity and acceleration to the closes link of the end-effector
	closest_link_velocity_["foot"] = id_.getVelocity_lowerleg();
	closest_link_acceleration_["foot"] = id_.getAcceleration_lowerleg();
}


HyLWholeBodyDynamics::~HyLWholeBodyDynamics()
{

}


void HyLWholeBodyDynamics::updateState(const rbd::Vector6d& base_pos, const Eigen::VectorXd& joint_pos)
{
	// Computing the HyL state
	Eigen::Vector3d state;
	state << base_pos(rbd::LZ), joint_pos;

	// Updating the end-effector motion transform to the closest link
	closest_link_motion_tf_["foot"] = motion_tf_.fr_foot_X_fr_lowerleg(state);

	// Updating the velocity and acceleration to the closes link of the end-effector
	closest_link_velocity_["foot"] = id_.getVelocity_lowerleg();
	closest_link_acceleration_["foot"] = id_.getAcceleration_lowerleg();
}


void HyLWholeBodyDynamics::computeInverseDynamics(rbd::Vector6d& base_wrench,
												  Eigen::VectorXd& joint_forces,
												  const rbd::Vector6d& g,
												  const rbd::Vector6d& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  const rbd::Vector6d& base_vel,
												  const Eigen::VectorXd& joint_vel,
												  const rbd::Vector6d& base_acc,
												  const Eigen::VectorXd& joint_acc)
{
	// HyL model defines the slider as actuated joint, which is the floating-base. Therefore, the floating-base wrench,
	// velocity and acceleration is converted as joint variables to the ID algorithm
	Eigen::Vector3d tau, q_pos, q_vel, q_acc;
	q_pos << base_pos(rbd::LZ), joint_pos;
	q_vel << base_vel(rbd::LZ), joint_vel;
	q_acc << base_acc(rbd::LZ), joint_acc;

	// Computing the inverse dynamics using the generated code of HyL
	id_.id(tau, q_pos, q_vel, q_acc);//const ExtForces& fext = zeroExtForces)

	// HyL only generate z force in the base
	base_wrench = iit::rbd::Vector6D::Zero();
	base_wrench(rbd::LZ) = tau(0);

	// Joint forces according the ID model
	joint_forces = tau.tail(2);
}


void HyLWholeBodyDynamics::propagateInverseDynamics(const rbd::Vector6d& base_pos,
													const Eigen::VectorXd& joint_pos,
													const rbd::Vector6d& base_vel,
													const Eigen::VectorXd& joint_vel,
													const rbd::Vector6d& base_acc,
													const Eigen::VectorXd& joint_acc)
{
	// HyL model defines the slider as actuated joint, which is the floating-base. Therefore, the floating-base wrench,
	// velocity and acceleration is converted as joint variables to the ID algorithm
	Eigen::Vector3d tau, jnt_pos, jnt_vel, jnt_acc;
	jnt_pos << base_pos(rbd::LZ), joint_pos;
	jnt_vel << base_vel(rbd::LZ), joint_vel;
	jnt_acc << base_acc(rbd::LZ), joint_acc;

	id_.setJointStatus(jnt_pos);
	id_.propagateVelAcc(jnt_vel, jnt_acc);
}

} //@namespace robot
} //namespace dwl
