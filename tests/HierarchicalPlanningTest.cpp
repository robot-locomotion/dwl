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
	dwl::model::WholeBodyKinematics kin;
	dwl::model::WholeBodyDynamics dyn;

	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyq_fb.urdf";
	kin.modelFromURDFFile(model_file, true);
	dyn.modelFromURDFFile(model_file);

	dwl::rbd::Vector6d base_wrench, base_pos, base_vel, base_acc;
	Eigen::VectorXd joint_forces(12), joint_pos(12), joint_vel(12), joint_acc(12);
	base_pos = dwl::rbd::Vector6d::Zero();
	base_vel = dwl::rbd::Vector6d::Zero();
	base_acc = dwl::rbd::Vector6d::Zero();
	base_pos << 0., 0., 0., 0., 0., 0.;
	base_vel << 0., 0., 0., 0., 0., 0.;
	base_acc << 0., 0., 0., 0., 0., 0.;
	joint_pos << 0., 0.75, -1.5, 0., -0.75, 1.5, 0., 0.75, -1.5, 0., -0.75, 1.5;
	joint_vel << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;//= Eigen::VectorXd::Zero(12);
	joint_acc << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;//= Eigen::VectorXd::Zero(12);


	// Defining contacts
	dwl::rbd::BodySelector contacts;
	contacts.push_back("lf_foot");
	contacts.push_back("lh_foot");
	contacts.push_back("rf_foot");
	contacts.push_back("rh_foot");
//	contacts.push_back("foot");


	// Computing the jacobians
	Eigen::MatrixXd jacobian, fixed_jac, floating_jac;
	kin.computeJacobian(jacobian, base_pos, joint_pos, contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacobian << " = jacobian" << std::endl;
	kin.getFixedBaseJacobian(fixed_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << fixed_jac << " = fixed jacobian" << std::endl;
	kin.getFloatingBaseJacobian(floating_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << floating_jac << " = floating jacobian" << std::endl;


	// Computing forward kinematics
	dwl::rbd::BodyVector fk_pos;
	kin.computeForwardKinematics(fk_pos,
								 base_pos, joint_pos,
								 contacts, dwl::rbd::Linear, dwl::RollPitchYaw);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << fk_pos["lf_foot"].transpose() << " = lf_foot tf" << std::endl;
	std::cout << fk_pos["rf_foot"].transpose() << " = rf_foot tf" << std::endl;
	std::cout << fk_pos["lh_foot"].transpose() << " = lh_foot tf" << std::endl;
	std::cout << fk_pos["rh_foot"].transpose() << " = rh_foot tf" << std::endl;


	// Computing inverse kinematics
	dwl::rbd::BodyPosition ik_pos;
	ik_pos["trunk"] = Eigen::Vector3d::Zero();
	ik_pos["lf_foot"] = fk_pos["lf_foot"].tail(3);
	ik_pos["rf_foot"] = fk_pos["rf_foot"].tail(3);
	ik_pos["lh_foot"] = fk_pos["lh_foot"].tail(3);
	ik_pos["rh_foot"] = fk_pos["rh_foot"].tail(3);
	dwl::rbd::Vector6d base_pos_init = dwl::rbd::Vector6d::Zero();
	base_pos_init(dwl::rbd::LZ) = 0.1;
	Eigen::VectorXd joint_pos_init(12);
	joint_pos_init << 0., 0.75, -1.5, 0., -0.75, 1.5, 0., 0.75, -1.5, 0., -0.75, 1.5;//
//	joint_pos_init = Eigen::VectorXd::Zero(12);//<< 0.2, -1;
	kin.computeInverseKinematics(base_pos, joint_pos,
								 base_pos_init, joint_pos_init,
								 ik_pos);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << base_pos.transpose() << " | " << joint_pos.transpose() << " = Body ik" << std::endl;


	// Computing body velocity
	dwl::rbd::BodyVector velocity;
	kin.computeVelocity(velocity,
						base_pos, joint_pos,
						base_vel, joint_vel,
						contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << velocity["lf_foot"].transpose() << " = Body vel" << std::endl;


	// Computing body acceleration
	dwl::rbd::BodyVector acceleration;
	kin.computeAcceleration(acceleration,
							base_pos, joint_pos,
							base_vel, joint_vel,
							base_acc, joint_acc,
							contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << acceleration["lf_foot"].transpose() << " = Body acc" << std::endl;


	// Computing body Jacd*Qd
	dwl::rbd::BodyVector jacd_qd;
	kin.computeJdotQdot(jacd_qd,
						base_pos, joint_pos,
						base_vel, joint_vel,
						contacts, dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacd_qd["lf_foot"].transpose() << " = jacd_qd" << std::endl;


	// Computing the ID
	dwl::rbd::BodyWrench grf;
//	grf["foot"] << 0, 0, 0, 0, 0, 67.3149;
//	grf["foot"] << 0, 0, 0, -15.2036, 0, 74.0133;
	grf["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rh_foot"] << 0, 0, 0, 0, 0, 190.778;
	dyn.computeInverseDynamics(base_wrench, joint_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc, grf);
	std::cout << "------------------ ID ---------------------" << std::endl;
	std::cout << "Base wrc = " << base_wrench.transpose() << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl << std::endl;



	// Estimating contact forces
	double force_threshold = 0;
	dwl::rbd::BodySelector active_contacts;
	dwl::rbd::BodyWrench contact_forces;
	dyn.estimateActiveContacts(active_contacts, contact_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc,
							   joint_forces, contacts, force_threshold);
	std::cout << "-------------------- estimated contact forces -------------------" << std::endl;
	std::cout << "lf_foot contact for = " << contact_forces.find("lf_foot")->second.transpose() << std::endl;
	std::cout << "rf_foot contact for = " << contact_forces.find("rf_foot")->second.transpose() << std::endl;
	std::cout << "lh_foot contact for = " << contact_forces.find("lh_foot")->second.transpose() << std::endl;
	std::cout << "rh_foot contact for = " << contact_forces.find("rh_foot")->second.transpose() << std::endl << std::endl;


	dyn.computeContactForces(contact_forces, joint_forces,
							 base_pos, joint_pos,
							 base_vel, joint_vel,
							 base_acc, joint_acc,
							 contacts);
	std::cout << "-------------------- contact forces -------------------" << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;
	std::cout << "lf_foot contact for = " << contact_forces.find("lf_foot")->second.transpose() << std::endl;
	std::cout << "rf_foot contact for = " << contact_forces.find("rf_foot")->second.transpose() << std::endl;
	std::cout << "lh_foot contact for = " << contact_forces.find("lh_foot")->second.transpose() << std::endl;
	std::cout << "rh_foot contact for = " << contact_forces.find("rh_foot")->second.transpose() << std::endl << std::endl;


	dyn.computeInverseDynamics(base_wrench, joint_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc, contact_forces);
	std::cout << "------------------ ID ---------------------" << std::endl;
	std::cout << "Base wrc = " << base_wrench.transpose() << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl << std::endl;


	// Computing the floating-base ID
	dyn.computeFloatingBaseInverseDynamics(base_acc, joint_forces,
										   base_pos, joint_pos,
										   base_vel, joint_vel,
										   joint_acc, contact_forces);
	std::cout << "------------------- floating-base ID --------------------" << std::endl;
	std::cout << "Base acc = " << base_acc.transpose() << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl << std::endl;






	// Computing the constrained ID
	base_acc.setZero();
	dyn.computeConstrainedFloatingBaseInverseDynamics(joint_forces,
													  base_pos, joint_pos,
													  base_vel, joint_vel,
													  base_acc, joint_acc,
													  contacts);
	std::cout << "------------------- constrained ID --------------------" << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;


    return 0;
}
