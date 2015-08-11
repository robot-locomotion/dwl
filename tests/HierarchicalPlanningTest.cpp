#include <model/WholeBodyKinematics.h>
#include <model/WholeBodyDynamics.h>
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
	dwl::model::FloatingBaseJoint joint(true, 0, "test_joint");
	dwl::model::FloatingBaseSystem system;
	system.setFloatingBaseJoint(joint, dwl::rbd::LZ);


	dwl::model::WholeBodyKinematics kin;
	kin.modelFromURDFFile(model_file, true);
	dwl::model::WholeBodyDynamics dyn;
	dyn.modelFromURDFFile(model_file);



	dwl::rbd::Vector6d base_wrench, base_pos, base_vel, base_acc;
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



	// Defining contacts
	dwl::rbd::BodySelector contacts;
//	contacts.push_back("lf_foot");
//	contacts.push_back("lh_foot");
//	contacts.push_back("rf_foot");
//	contacts.push_back("rh_foot");
	contacts.push_back("foot");


	// Computing the jacobians
	Eigen::MatrixXd jacobian, fixed_jac, floating_jac;
	kin.computeJacobian(jacobian, base_pos, joint_pos, contacts, dwl::rbd::Full);
	kin.getFixedBaseJacobian(fixed_jac, jacobian);
	kin.getFloatingBaseJacobian(floating_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacobian << " = jacobian" << std::endl;
	std::cout << "---------------------------------------" << std::endl;
	std::cout << fixed_jac << " = fixed jacobian" << std::endl;
	std::cout << "---------------------------------------" << std::endl;
	std::cout << floating_jac << " = floating jacobian" << std::endl;


	// Computing forward kinematics
	dwl::rbd::BodyVector fk_pos;
	kin.computeForwardKinematics(fk_pos, base_pos, joint_pos, contacts, dwl::rbd::Linear, dwl::RollPitchYaw);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << fk_pos["foot"].transpose() << " = Body tf" << std::endl;


	// Computing inverse kinematics
	dwl::rbd::BodyPosition ik_pos;
	ik_pos["trunk"] = Eigen::Vector3d::Zero();
	ik_pos["foot"] = fk_pos["foot"].tail(3);
	dwl::rbd::Vector6d base_pos_init = dwl::rbd::Vector6d::Zero();
	base_pos_init(dwl::rbd::LZ) = 0.1;
	Eigen::VectorXd joint_pos_init(2);// = Eigen::VectorXd::Zero(2);
	joint_pos_init << 0.2, -1;
	kin.computeInverseKinematics(base_pos, joint_pos, base_pos_init, joint_pos_init, ik_pos);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << base_pos.transpose() << " | " << joint_pos.transpose() << " = Body ik" << std::endl;


	// Computing body velocity
	dwl::rbd::BodyVector velocity;
	kin.computeVelocity(velocity, base_pos, joint_pos, base_vel, joint_vel, contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << velocity["foot"].transpose() << " = Body vel" << std::endl;


	// Computing body acceleration
	dwl::rbd::BodyVector acceleration;
	kin.computeAcceleration(acceleration, base_pos, joint_pos, base_vel, joint_vel, base_acc, joint_acc, contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << acceleration["foot"].transpose() << " = Body acc" << std::endl;


	// Computing body Jacd*Qd
	dwl::rbd::BodyVector jacd_qd;
	kin.computeJdotQdot(jacd_qd, base_pos, joint_pos, base_vel, joint_vel, contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacd_qd["foot"].transpose() << " = jacd_qd" << std::endl;


	// Computing the ID
	dwl::rbd::BodyWrench grf;
	grf["foot"] << 0, 0, 0, 0, 0, 67.3149;
	dyn.computeInverseDynamics(base_wrench, joint_forces, base_pos, joint_pos, base_vel, joint_vel, base_acc, joint_acc, grf);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << base_wrench.transpose() << " | " << joint_forces.transpose() << " = tau" << std::endl;


	// Computing the constrained ID
	dyn.computeConstrainedFloatingBaseInverseDynamics(joint_forces, base_pos, joint_pos,
													  base_vel, joint_vel, base_acc, joint_acc,
													  contacts);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << "Base acc = " << base_acc.transpose() << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;


    return 0;
}
