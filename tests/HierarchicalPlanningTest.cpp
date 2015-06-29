#include <model/WholeBodyKinematics.h>
#include <model/WholeBodyDynamics.h>
//#include <robot/HyLWholeBodyKinematics.h>
//#include <robot/HyLWholeBodyDynamics.h>
#include <iit/rbd/rbd.h>


void get(Eigen::VectorXd& constraint, Eigen::VectorXd state)
{
	constraint = 2 * state;
	std::cout << "get = " << constraint.transpose() << std::endl;
}


int main(int argc, char **argv)
{
/*	dwl::model::WholeBodyKinematics* kin_ptr = new dwl::robot::HyLWholeBodyKinematics();
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

*/

	Eigen::VectorXd all_constraint(4);
	all_constraint.setZero();

	double x_raw[] = {1,2};
	const double* x = x_raw;
	for (int i = 0; i < 2; i++)
		std::cout << x[i] << " ";
	std::cout << std::endl;

	Eigen::Map<const Eigen::VectorXd> state(x, 2);
	Eigen::VectorXd constraint;
	get(constraint, (Eigen::VectorXd) state);
	std::cout << "size " << constraint.size() << std::endl;

	all_constraint.segment(0,2) = constraint;
	all_constraint.segment(2,2) = constraint;

	std::cout << all_constraint.transpose() << std::endl;

	double* data;
	data = all_constraint.data();
	for (int i = 0; i < 5; i++)
		std::cout << data[i] << " ";
	std::cout << std::endl;


    return 0;
}
