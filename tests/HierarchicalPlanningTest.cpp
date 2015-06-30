#include <model/WholeBodyKinematics.h>
#include <model/WholeBodyDynamics.h>
#include <model/RobCoGenWholeBodyKinematics.h>
#include <model/RobCoGenWholeBodyDynamics.h>
#include <robot/HyLWholeBodyKinematics.h>
#include <robot/HyLWholeBodyDynamics.h>
#include <iit/rbd/rbd.h>
#include "ctime"

struct TimerInfo {
	/// time stamp when timer_start() gets called
	clock_t clock_start_value;

	/// time stamp when the timer was stopped
	clock_t clock_end_value;

	/// duration between clock_start_value and clock_end_value in seconds
	double duration_sec;
};

inline void timer_start (TimerInfo *timer) {
	timer->clock_start_value = clock();
}

inline double timer_stop (TimerInfo *timer) {
	timer->clock_end_value = clock();

	timer->duration_sec = static_cast<double>(timer->clock_end_value - timer->clock_start_value) * 1 / CLOCKS_PER_SEC;

	return timer->duration_sec;
}


int main(int argc, char **argv)
{

	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	dwl::rbd::ReducedFloatingBase reduced_base;
	reduced_base.TZ.active = true;
	reduced_base.TZ.id = 0;

	dwl::model::WholeBodyKinematics kin;
	kin.modelFromURDFFile(model_file, &reduced_base, true);
	dwl::model::WholeBodyDynamics dyn;
	dyn.modelFromURDFFile(model_file, &reduced_base);

	dwl::model::RobCoGenWholeBodyKinematics* kin_ptr = new dwl::robot::HyLWholeBodyKinematics();
	dwl::model::RobCoGenWholeBodyDynamics* dyn_ptr = new dwl::robot::HyLWholeBodyDynamics();
	dyn_ptr->setKinematicModel(kin_ptr);

	iit::rbd::Vector6D base_wrench, base_pos, base_vel, base_acc, g;
	Eigen::VectorXd joint_forces(2), joint_pos(2), joint_vel(2), joint_acc(2);
	base_pos = dwl::rbd::Vector6d::Zero();
	base_vel = dwl::rbd::Vector6d::Zero();
	base_acc = dwl::rbd::Vector6d::Zero();
	base_pos << 0., 0., 0., 0., 0., 0.;
	base_vel << 0., 0., 0., 0., 0., 0.;
	base_acc << 0., 0., 0., 0., 0., 0.;
	joint_pos << 0.6, -1.5;//, 0., -0.75, 1.5, 0., 0.75, -1.5, 0., -0.75, 1.5;
	joint_vel << 0., 0.;//, 0., 0., 0., 0., 0., 0., 0., 0., 0.;//= Eigen::VectorXd::Zero(12);
	joint_acc << 0., 0.;//, 0., 0., 0., 0., 0., 0., 0., 0., 0.;//= Eigen::VectorXd::Zero(12);


	// Updating kinematics and dynamics
//	kin_ptr->updateState(base_pos, joint_pos);
//	dyn_ptr->updateState(base_pos, joint_pos);

	// Defining contacts
	dwl::rbd::BodySelector contacts;
//	contacts.push_back("lf_foot");
//	contacts.push_back("lh_foot");
//	contacts.push_back("rf_foot");
//	contacts.push_back("rh_foot");
	contacts.push_back("foot");
//	contacts.push_back("foot");


	// Computing the jacobians
//	Eigen::MatrixXd jacobian, fixed_jac, floating_jac;
//	kin.computeJacobian(jacobian, base_pos, joint_pos, contacts, dwl::rbd::Full);
//	kin.getFixedBaseJacobian(fixed_jac, jacobian);
//	kin.getFloatingBaseJacobian(floating_jac, jacobian);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << jacobian << " = jacobian" << std::endl;
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << fixed_jac << " = fixed jacobian" << std::endl;
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << floating_jac << " = floating jacobian" << std::endl;

	// Computing forward kinematics
//	Eigen::VectorXd fk_pos;
//	kin.computeForwardKinematics(fk_pos, base_pos, joint_pos, contacts, dwl::rbd::Linear, dwl::RollPitchYaw);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << fk_pos << " = Body tf" << std::endl;

/*
	dwl::rbd::EndEffectorPosition ik_pos;
	ik_pos["trunk"] = Eigen::Vector3d::Zero();
	ik_pos["foot"] = fk_pos.tail(3);
	dwl::rbd::Vector6d base_pos_init = dwl::rbd::Vector6d::Zero();
	base_pos_init(dwl::rbd::TZ) = 0.1;
	Eigen::VectorXd joint_pos_init(2);// = Eigen::VectorXd::Zero(2);
	joint_pos_init << 0.2, -1;
	kin.computeInverseKinematics(base_pos, joint_pos, base_pos_init, joint_pos_init, ik_pos);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << base_pos.transpose() << " | " << joint_pos.transpose() << " = Body ik" << std::endl;
	std::cout << joint_pos_init.transpose() << " = jnt init" << std::endl;
*/



//	Eigen::VectorXd position;
//	kin_ptr->computeForwardKinematics(position, joint_pos, dwl::model::Linear);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << position << " = MarcoF tf" << std::endl;

//	Eigen::VectorXd velocity;
//	kin.computeVelocity(velocity, base_pos, joint_pos, base_vel, joint_vel, contacts, dwl::rbd::Full);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << velocity << " = Body vel" << std::endl;

//	kin_ptr->opVelocityFromJointSpace(velocity, base_pos, joint_pos, base_vel, joint_vel, effector_set, dwl::Full);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << velocity << " = robcogen Body vel" << std::endl;

//	Eigen::VectorXd acceleration;
//	kin.computeAcceleration(acceleration, base_pos, joint_pos, base_vel, joint_vel, base_acc, joint_acc, contacts, dwl::rbd::Full);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << acceleration << " = Body acc" << std::endl;




	Eigen::VectorXd jacd_qd;
	kin.computeJdotQdot(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacd_qd.transpose() << " = jacd_qd" << std::endl;


//	dwl::rbd::EndEffectorForce fext;
//	dwl::rbd::Vector6d forc;
//	forc << 0,0,0,0,0,10;//67;
//	fext["trunk"] = forc;
//	fext["foot"] = forc;
//	fext["lowerleg"] = forc;



//	base_pos(dwl::rbd::AX) = 1.57;
//	base_pos(dwl::rbd::AY) = 1.57;
	//base_pos(dwl::rbd::AZ) = 3;
//	base_pos(dwl::rbd::LX) = 10;
//	base_pos(dwl::rbd::LY) = 20;
//	base_pos(dwl::rbd::LZ) = 30;
//	base_acc(dwl::rbd::LZ) = -9.81;
//	base_vel(dwl::rbd::LX) = 1;
//	base_vel(dwl::rbd::LZ) = 2;
//	joint_vel2 << 1;
//	joint_acc2 << -9.81;

	dwl::rbd::BodyForce grf;
	grf["foot"] << 0, 0, 0, 0, 0, 67.3149;
//	dyn.computeWholeDynamics(base_wrench, joint_forces, base_pos, joint_pos2, base_vel, joint_vel2, base_acc, joint_acc2, fext);
//	std::cout << "---------------------------------------" << std::endl;
//	std::cout << base_wrench.transpose() << " | " << joint_forces.transpose() << " = tau" << std::endl;

//	dyn_ptr->updateState(base_pos, joint_pos);
//	TimerInfo tinfo1, tinfo2;
//	timer_start (&tinfo1);
//	for (int i = 0; i < 1000000; i++)
//		dyn_ptr->computeInverseDynamics(base_wrench, joint_forces, dwl::rbd::Vector6d::Zero(), base_pos, joint_pos, base_vel, joint_vel, base_acc, joint_acc, grf);
//	std::cout << "---------------------------------------" << std::endl;
//	double duration = timer_stop (&tinfo1);
//	std::cout << duration << " = time duration" << std::endl;
//	std::cout << base_wrench.transpose() << " | " << joint_forces.transpose() << " = robcogen tau" << std::endl;
//	base_wrench.setZero();
//	joint_forces.setZero();
//	timer_start (&tinfo2);
//	for (int i = 0; i < 1000000; i++)
//		dyn.computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos, base_vel, joint_vel, base_acc, joint_acc, grf);
	std::cout << "---------------------------------------" << std::endl;
//	duration = timer_stop (&tinfo2);
//	std::cout << duration << " = time duration" << std::endl;
//	std::cout << base_wrench.transpose() << " | " << joint_forces.transpose() << " = rbdl tau" << std::endl;
//
//
//	std::cout << "Base acc = " << base_acc.transpose() << std::endl;
//	std::cout << "Base wrench = " << base_wrench.transpose() << std::endl;
//	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;

///*

	dyn.computeConstrainedFloatingBaseInverseDynamics(joint_forces, base_pos, joint_pos,
													  base_vel, joint_vel, base_acc, joint_acc,
													  contacts);
	std::cout << "Base acc = " << base_acc.transpose() << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;
//*/

    return 0;
}
