#include <model/WholeBodyKinematics.h>
#include <model/WholeBodyDynamics.h>
#include <robot/HyLWholeBodyKinematics.h>
#include <robot/HyLWholeBodyDynamics.h>
#include <iit/rbd/rbd.h>




int main(int argc, char **argv)
{
	dwl::model::WholeBodyKinematics* kin_ptr = new dwl::robot::HyLWholeBodyKinematics();
	dwl::model::WholeBodyDynamics* dyn_ptr = new dwl::robot::HyLWholeBodyDynamics();
	dyn_ptr->setKinematicModel(kin_ptr);

	iit::rbd::Vector6D base_wrench, base_pos, base_vel, base_acc, g;
	Eigen::VectorXd joint_forces(2), joint_pos(2), joint_vel(2), joint_acc(2);

	base_pos = iit::rbd::Vector6D::Zero();
	base_vel = iit::rbd::Vector6D::Zero();
	base_acc = iit::rbd::Vector6D::Zero();
	joint_pos << 0.75, -1.5;
	joint_vel << 0., 0.;
	joint_acc << 0., 0.;


	// Testing kinematics
	kin_ptr->init();
	kin_ptr->updateState(base_pos, joint_pos);

	Eigen::VectorXd effector_pos;
	kin_ptr->computeEffectorFK(effector_pos, dwl::model::Full);
	std::cout << effector_pos << std::endl;
	std::cout << "---------------------------------------" << std::endl;


	dwl::EndEffectorSelector effector_set;
	effector_set["foot"] = true;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;
//	kin_ptr->computeFloatingBaseJacobian(jacobian, dwl::model::Full);
//	kin_ptr->computeFixedBaseJacobian(jacobian, active_contact, dwl::model::Full);
	kin_ptr->computeWholeBodyJacobian(jacobian, effector_set, dwl::model::Full);
	std::cout << jacobian << std::endl;
	std::cout << "---------------------------------------" << std::endl;

	// Testing dynamics
	Eigen::VectorXd jacd_qd;
	dyn_ptr->init();
	dyn_ptr->updateState(base_pos, joint_pos);
	dyn_ptr->computeJointVelocityContributionOfAcceleration(jacd_qd, base_pos, base_vel, joint_pos, joint_vel);
	std::cout << "jacd_qd = " << jacd_qd.transpose() << std::endl;
	std::cout << "jacd_qd2 = " << jacd_qd.transpose().tail(2) << std::endl;

	dyn_ptr->computeWholeBodyInverseDynamics(base_wrench, joint_forces, g, base_pos, base_vel, base_acc, joint_pos, joint_vel, joint_acc);
	Eigen::Vector3d tau;
	tau << base_wrench(iit::rbd::LZ), joint_forces;
	std::cout << "tau = " << tau.transpose() << std::endl;


    return 0;
}
