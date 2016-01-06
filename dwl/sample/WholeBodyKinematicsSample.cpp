#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/model/WholeBodyKinematics.h>

using namespace std;



int main(int argc, char **argv)
{
	dwl::model::FloatingBaseSystem sys;
	dwl::model::WholeBodyKinematics kin;

	// Resetting the system from the hyq urdf file
	std::string model_file = "../sample/hyq.urdf";
	sys.resetFromURDFFile(model_file);
	kin.modelFromURDFFile(model_file);


	// The robot state
	dwl::rbd::Vector6d base_wrench, base_pos, base_vel, base_acc;
	Eigen::VectorXd joint_forces(12), joint_pos(12), joint_vel(12), joint_acc(12);
	base_pos = dwl::rbd::Vector6d::Zero();
	base_vel = dwl::rbd::Vector6d::Zero();
	base_acc = dwl::rbd::Vector6d::Zero();
	base_wrench = dwl::rbd::Vector6d::Zero();
	joint_pos = Eigen::VectorXd::Zero(12);
	joint_pos(sys.getJointId("lf_hfe_joint")) = 0.75;
	joint_pos(sys.getJointId("lf_kfe_joint")) = -1.5;
	joint_pos(sys.getJointId("lh_hfe_joint")) = -0.75;
	joint_pos(sys.getJointId("lh_kfe_joint")) = 1.5;
	joint_pos(sys.getJointId("rf_hfe_joint")) = 0.75;
	joint_pos(sys.getJointId("rf_kfe_joint")) = -1.5;
	joint_pos(sys.getJointId("rh_hfe_joint")) = -0.75;
	joint_pos(sys.getJointId("rh_kfe_joint")) = 1.5;
	joint_vel = Eigen::VectorXd::Zero(12);
	joint_acc = Eigen::VectorXd::Zero(12);



	// Computing the jacobians
	Eigen::MatrixXd jacobian, fixed_jac, floating_jac;
	kin.computeJacobian(jacobian,
						base_pos, joint_pos,
						sys.getEndEffectorNames(),
						dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacobian << " = jacobian" << std::endl;
	kin.getFixedBaseJacobian(fixed_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << fixed_jac << " = fixed jacobian" << std::endl;
	kin.getFloatingBaseJacobian(floating_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << floating_jac << " = floating jacobian" << std::endl;


	// Computing FK
	dwl::rbd::BodyVector contact_pos;
	kin.computeForwardKinematics(contact_pos,
								 base_pos, joint_pos,
								 sys.getEndEffectorNames(), // it uses all the end-effector of the system
								 dwl::rbd::Linear, dwl::RollPitchYaw);
	cout << "------------------- FK --------------------" << endl;
	for (dwl::rbd::BodyVector::iterator it = contact_pos.begin();
			it != contact_pos.end(); it++) {
		string name = it->first;
		Eigen::VectorXd position = it->second;

		cout << "fk[" << name << "] = " << position.transpose() << endl << endl;
	}



	// Computing IK
	dwl::rbd::BodyPosition ik_pos;
	ik_pos[sys.getFloatingBaseName()] = Eigen::Vector3d::Zero();
	ik_pos["lf_foot"] = contact_pos["lf_foot"].tail(3);
	ik_pos["rf_foot"] = contact_pos["rf_foot"].tail(3);
	ik_pos["lh_foot"] = contact_pos["lh_foot"].tail(3);
	ik_pos["rh_foot"] = contact_pos["rh_foot"].tail(3);
	dwl::rbd::Vector6d base_pos_init = dwl::rbd::Vector6d::Zero();
	Eigen::VectorXd joint_pos_init(12);
	joint_pos_init << 0., 0.5, -1., 0., -0.5, 1., 0., 0.5, -1., 0., -0.5, 1.;
	kin.computeInverseKinematics(base_pos, joint_pos,
								 base_pos_init, joint_pos_init,
								 ik_pos);
	cout << "------------------ IK ---------------------" << endl;
	cout << "Base position = " << base_pos.transpose() << endl;
	cout << "Joint positions = "<< joint_pos.transpose() << endl << endl;


	// Computing the contact velocities
	dwl::rbd::BodyVector contact_vel;
	kin.computeVelocity(contact_vel,
						base_pos, joint_pos,
						base_vel, joint_vel,
						sys.getEndEffectorNames(), dwl::rbd::Full);
	cout << "------------------- Contact velocities --------------------" << endl;
	for (dwl::rbd::BodyVector::iterator it = contact_vel.begin();
			it != contact_vel.end(); it++) {
		string name = it->first;
		Eigen::VectorXd velocity = it->second;

		cout << "velocity[" << name << "] = " << velocity.transpose() << endl << endl;
	}


	// Computing the contact accelerations
	dwl::rbd::BodyVector contact_acc;
	kin.computeAcceleration(contact_acc,
							base_pos, joint_pos,
							base_vel, joint_vel,
							base_acc, joint_acc,
							sys.getEndEffectorNames(), dwl::rbd::Full);
	cout << "------------------- Contact accelerations --------------------" << endl;
	for (dwl::rbd::BodyVector::iterator it = contact_acc.begin();
			it != contact_acc.end(); it++) {
		string name = it->first;
		Eigen::VectorXd acceleration = it->second;

		cout << "acceleration[" << name << "] = " << acceleration.transpose() << endl << endl;
	}


	// Computing the contact Jacd*Qd
	dwl::rbd::BodyVector contact_jacd_qd;
	kin.computeJdotQdot(contact_jacd_qd,
						base_pos, joint_pos,
						base_vel, joint_vel,
						sys.getEndEffectorNames(), dwl::rbd::Full);
	cout << "------------------- Contact Jdot*Qdot --------------------" << endl;
	for (dwl::rbd::BodyVector::iterator it = contact_jacd_qd.begin();
			it != contact_jacd_qd.end(); it++) {
		string name = it->first;
		Eigen::VectorXd jacd_qd = it->second;

		cout << "jacd_qd[" << name << "] = " << jacd_qd.transpose() << endl << endl;
	}


    return 0;
}
