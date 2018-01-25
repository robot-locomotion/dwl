#include <dwl/WholeBodyState.h>
#include <dwl/model/WholeBodyKinematics.h>



int main(int argc, char **argv)
{
	dwl::WholeBodyState ws;
	dwl::model::FloatingBaseSystem fbs;
	dwl::model::WholeBodyKinematics wkin;

	// Resetting the system from the hyq urdf file
	std::string urdf_file = DWL_SOURCE_DIR"/sample/hyq.urdf";
	std::string yarf_file = DWL_SOURCE_DIR"/config/hyq.yarf";
	fbs.resetFromURDFFile(urdf_file, yarf_file);
	wkin.modelFromURDFFile(urdf_file, yarf_file);

	// Define the DoF after initializing the robot model
	ws.setJointDoF(fbs.getJointDoF());


	// The robot state
	ws.setBasePosition(Eigen::Vector3d(0., 0., 0.));
	ws.setBaseRPY(Eigen::Vector3d(0., 0., 0.));
	ws.setBaseVelocity_W(Eigen::Vector3d(0., 0., 0.));
	ws.setBaseRPYVelocity_W(Eigen::Vector3d(0., 0., 0.));
	ws.setBaseAcceleration_W(Eigen::Vector3d(0., 0., 0.));
	ws.setBaseRPYAcceleration_W(Eigen::Vector3d(0., 0., 0.));
	ws.setJointPosition(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointPosition(-1.5, fbs.getJointId("lf_kfe_joint"));
	ws.setJointPosition(-0.75, fbs.getJointId("lh_hfe_joint"));
	ws.setJointPosition(1.5, fbs.getJointId("lh_kfe_joint"));
	ws.setJointPosition(0.75, fbs.getJointId("rf_hfe_joint"));
	ws.setJointPosition(-1.5, fbs.getJointId("rf_kfe_joint"));
	ws.setJointPosition(-0.75, fbs.getJointId("rh_hfe_joint"));
	ws.setJointPosition(1.5, fbs.getJointId("rh_kfe_joint"));


	// Computing the jacobians
	Eigen::MatrixXd jacobian, fixed_jac, floating_jac;
	wkin.computeJacobian(jacobian,
						 ws.base_pos, ws.joint_pos,
						 fbs.getEndEffectorNames(),
						 dwl::rbd::Full);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << jacobian << " = jacobian" << std::endl;
	wkin.getFixedBaseJacobian(fixed_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << fixed_jac << " = fixed jacobian" << std::endl;
	wkin.getFloatingBaseJacobian(floating_jac, jacobian);
	std::cout << "---------------------------------------" << std::endl;
	std::cout << floating_jac << " = floating jacobian" << std::endl << std::endl;


	// Computing contact positions w.r.t to the world frame
	dwl::rbd::BodyVectorXd contact_pos_W;
	wkin.computeForwardKinematics(contact_pos_W,
								  ws.base_pos, ws.joint_pos,
								  fbs.getEndEffectorNames(), // it uses all the end-effector of the system
								  dwl::rbd::Linear, dwl::RollPitchYaw);
	ws.setContactPosition_W(contact_pos_W);
	std::cout << "------------------- Contact positions --------------------" << std::endl;
	for (unsigned int c = 0; c < fbs.getNumberOfEndEffectors(); ++c) {
		std::string name = fbs.getEndEffectorNames()[c];
		Eigen::VectorXd position = ws.getContactPosition_W(name);
		std::cout << name << " = " << position.transpose() << std::endl << std::endl;
	}


	// Computing the contact velocities w.r.t to the world frame
	dwl::rbd::BodyVectorXd contact_vel_W;
	wkin.computeVelocity(contact_vel_W,
						 ws.base_pos, ws.joint_pos,
						 ws.base_vel, ws.joint_vel,
						 fbs.getEndEffectorNames(), dwl::rbd::Full);
	ws.setContactVelocity_W(contact_vel_W);
	std::cout << "------------------- Contact velocities --------------------" << std::endl;
	for (unsigned int c = 0; c < fbs.getNumberOfEndEffectors(); ++c) {
		std::string name = fbs.getEndEffectorNames()[c];
		Eigen::VectorXd velocity = ws.getContactVelocity_W(name);
		std::cout << name << " = " << velocity.transpose() << std::endl << std::endl;
	}


	// Computing the contact accelerations w.r.t to the world frame
	dwl::rbd::BodyVectorXd contact_acc_W;
	wkin.computeAcceleration(contact_acc_W,
							 ws.base_pos, ws.joint_pos,
							 ws.base_vel, ws.joint_vel,
							 ws.base_acc, ws.joint_acc,
							 fbs.getEndEffectorNames(), dwl::rbd::Full);
	ws.setContactAcceleration_W(contact_acc_W);
	std::cout << "------------------- Contact accelerations --------------------" << std::endl;
	for (unsigned int c = 0; c < fbs.getNumberOfEndEffectors(); ++c) {
		std::string name = fbs.getEndEffectorNames()[c];
		Eigen::VectorXd acceleration = ws.getContactAcceleration_W(name);
		std::cout << name << " = " << acceleration.transpose() << std::endl << std::endl;
	}


	// Computing IK
	wkin.setIKSolver( 1.0e-12, 0.01, 50);
	dwl::rbd::BodyVector3d ik_pos;
	ik_pos[fbs.getFloatingBaseName()] = Eigen::Vector3d::Zero();
	ik_pos["lf_foot"] = ws.getContactPosition_W("lf_foot").tail(3);
	ik_pos["rf_foot"] = ws.getContactPosition_W("rf_foot").tail(3);
	ik_pos["lh_foot"] = ws.getContactPosition_W("lh_foot").tail(3);
	ik_pos["rh_foot"] = ws.getContactPosition_W("rh_foot").tail(3);
	dwl::rbd::Vector6d base_pos_init = dwl::rbd::Vector6d::Zero();
	Eigen::VectorXd joint_pos_init(12);
	joint_pos_init << 0., 0.5, -1., 0., -0.5, 1., 0., 0.5, -1., 0., -0.5, 1.;
	std::cout << "------------------ WB-IK ---------------------" << std::endl;
	if (wkin.computeInverseKinematics(ws.base_pos, ws.joint_pos,
									  ik_pos,
								 	  base_pos_init, joint_pos_init)) {
		std::cout << "Base position = " << ws.base_pos.transpose() << std::endl;
		std::cout << "Joint position = "<< ws.joint_pos.transpose() << std::endl << std::endl;
	} else {
		std::cout << "The WB-IK problem could not be solved" << std::endl;
	}

	std::cout << "------------------ IK ---------------------" << std::endl;
	if (wkin.computeJointPosition(ws.joint_pos,
								  ik_pos,
								  joint_pos_init)) {
		std::cout << "Joint position = "<< ws.joint_pos.transpose() << std::endl << std::endl;
	} else {
		std::cout << "The IK problem could not be solved" << std::endl;
	}


	// Computing the contact Jacd*Qd
	dwl::rbd::BodyVectorXd contact_jacd_qd;
	wkin.computeJdotQdot(contact_jacd_qd,
						 ws.base_pos, ws.joint_pos,
						 ws.base_vel, ws.joint_vel,
						 fbs.getEndEffectorNames(), dwl::rbd::Full);
	std::cout << "------------------- Contact Jdot*Qdot --------------------" << std::endl;
	for (dwl::rbd::BodyVectorXd::iterator it = contact_jacd_qd.begin();
			it != contact_jacd_qd.end(); ++it) {
		std::string name = it->first;
		Eigen::VectorXd jacd_qd = it->second;

		std::cout << name << " = " << jacd_qd.transpose() << std::endl << std::endl;
	}


    return 0;
}
