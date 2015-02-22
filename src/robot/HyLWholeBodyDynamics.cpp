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


void HyLWholeBodyDynamics::computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
        												   const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_vel,
        												   const iit::rbd::Vector6D& base_accel, const Eigen::VectorXd& q,
        												   const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd)
{
	// HyL model defines the slider as actuated joint, which is the floating-base. Therefore, the floating-base wrench,
	// velocity and acceleration is converted as joint variables to the ID algorithm
	Eigen::Vector4d tau, jnt_pos, jnt_vel, jnt_acc;
	jnt_pos << 0, q.head(3);//base_pos(iit::rbd::LZ)
	jnt_vel << base_vel(iit::rbd::LZ), qd.head(3);
	jnt_acc << base_accel(iit::rbd::LZ), qdd.head(3);

	// Computing the inverse dynamics using the generated code of HyL
	id_.id(tau, jnt_pos, jnt_vel, jnt_acc);//const ExtForces& fext = zeroExtForces)

	// HyL only generate z force in the base
	base_wrench = iit::rbd::Vector6D::Zero();
	base_wrench(iit::rbd::LZ) = tau(0);

	// Joint forces according the ID model
	joint_forces = tau.tail(3);


	std::cout << "Base wrench = " << base_wrench.transpose() << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;
}

} //@namespace robot
} //namespace dwl
