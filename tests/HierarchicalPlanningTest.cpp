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

	Eigen::VectorXd tau, q, qd, qdd;
	q = Eigen::VectorXd::Zero(2);
	q << 0.75, -1.5;
	qd = Eigen::VectorXd::Zero(2);
	qdd = Eigen::VectorXd::Zero(2);
	Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> base_vel = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> base_accel = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> wrench;
	Eigen::VectorXd state = Eigen::VectorXd::Zero(3);
	state << 0, q;
	Eigen::VectorXd state_dot = Eigen::VectorXd::Zero(3);

	// Testing kinematics
	kin_ptr->init();
	kin_ptr->updateState(state);

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
	dyn_ptr->computeJointVelocityContributionOfAcceleration(jacd_qd, state, state_dot);
	std::cout << jacd_qd << std::endl;
	std::cout << jacd_qd.tail(2) << std::endl;

//	dyn_ptr->computeWholeBodyInverseDynamics(wrench, tau, g, base_vel, base_accel, q, qd, qdd);

    return 0;
}
