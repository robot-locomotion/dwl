#include <dwl/WholeBodyState.h>
#include <dwl/model/WholeBodyDynamics.h>



int main(int argc, char **argv)
{
	dwl::WholeBodyState ws;
	dwl::model::FloatingBaseSystem fbs;
	dwl::model::WholeBodyDynamics wdyn;

	// Resetting the system from the hyq urdf file
	std::string urdf_file = "../sample/hyq.urdf";
	std::string yarf_file = "../config/hyq.yarf";
	fbs.resetFromURDFFile(urdf_file, yarf_file);
	wdyn.modelFromURDFFile(urdf_file, yarf_file);

	// Define the DoF after initializing the robot model
	ws.setJointDoF(fbs.getJointDoF());


	// The robot state
	ws.setBasePosition_W(Eigen::Vector3d(0., 0., 0.));
	ws.setBaseRPY_W(Eigen::Vector3d(0., 0., 0.));
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


	// Computing the ID
	dwl::rbd::BodyVector6d grf;
	grf["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rh_foot"] << 0, 0, 0, 0, 0, 190.778;
	wdyn.computeInverseDynamics(ws.base_eff, ws.joint_eff,
								ws.base_pos, ws.joint_pos,
								ws.base_vel, ws.joint_vel,
								ws.base_acc, ws.joint_acc, grf);
	std::cout << "--------------------------- ID --------------------------" << std::endl;
	std::cout << "Base wrench = " << ws.base_eff.transpose() << std::endl;
	std::cout << "Joint forces = " << ws.joint_eff.transpose() << std::endl << std::endl;


	// Estimating active contacts
	double force_threshold = 0.;
	dwl::rbd::BodySelector active_contacts;
	dwl::rbd::BodyVector6d contact_forces;
	wdyn.estimateActiveContactsAndForces(active_contacts, contact_forces,
								ws.base_pos, ws.joint_pos,
								ws.base_vel, ws.joint_vel,
								ws.base_acc, ws.joint_acc,
								ws.joint_eff, fbs.getEndEffectorNames(), // it uses all the end-effector of the system
								force_threshold);
	std::cout << "--------------- Estimated active contacts ---------------" << std::endl;
	for (dwl::rbd::BodyVector6d::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		std::string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		std::cout << "active contact: " << name << " and its force is " << force.transpose() << std::endl << std::endl;
	}


	// Estimating contact forces from joint measurements
	wdyn.computeContactForces(contact_forces, ws.joint_eff,
							  ws.base_pos, ws.joint_pos,
							  ws.base_vel, ws.joint_vel,
							  ws.base_acc, ws.joint_acc,
							  fbs.getEndEffectorNames());
	std::cout << "--------------- Estimated contact forces ----------------" << std::endl;
	std::cout << "Joint forces = " << ws.joint_eff.transpose() << std::endl;
	for (dwl::rbd::BodyVector6d::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		std::string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		std::cout << "contact force[" << name << "] = " << force.transpose() << std::endl;
	}
	std::cout << std::endl;


	// Computing the floating-base ID
	wdyn.computeFloatingBaseInverseDynamics(ws.base_acc, ws.joint_eff,
											ws.base_pos, ws.joint_pos,
											ws.base_vel, ws.joint_vel,
											ws.joint_acc, contact_forces);
	std::cout << "---------------- Floating-base ID ------------------" << std::endl;
	std::cout << "Base accelerations = " << ws.base_acc.transpose() << std::endl;
	std::cout << "Joint forces = " << ws.joint_eff.transpose() << std::endl << std::endl;


	// Computing the constrained ID
	ws.base_acc.setZero();
	wdyn.computeConstrainedFloatingBaseInverseDynamics(ws.joint_eff,
													   ws.base_pos, ws.joint_pos,
													   ws.base_vel, ws.joint_vel,
													   ws.base_acc, ws.joint_acc,
													   fbs.getEndEffectorNames());
	std::cout << "------------------ Constrained ID ------------------" << std::endl;
	std::cout << "Joint forces = " << ws.joint_eff.transpose() << std::endl << std::endl;


	// Computing the contact forces from the CoP
	Eigen::Vector3d cop_pos(0.0196, -0.0012, 0.0215);
	dwl::rbd::BodyVector3d feet_pos;
	feet_pos["lf_foot"] = Eigen::Vector3d(0.319215579664, 0.206424153349, 0.0215);
	feet_pos["lh_foot"] = Eigen::Vector3d(-0.335953242968, 0.207404146377, 0.0215);
	feet_pos["rf_foot"] = Eigen::Vector3d(0.31996232038, -0.207592286639, 0.0215);
	feet_pos["rh_foot"] = Eigen::Vector3d(-0.331894998575, -0.207236136594, 0.0215);
	wdyn.estimateGroundReactionForces(contact_forces,
									  cop_pos, feet_pos,
									  fbs.getEndEffectorNames(dwl::model::FOOT));
	std::cout << "----------------- Ground reaction forces from CoP ---------------" << std::endl;
	for (dwl::rbd::BodyVector6d::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		std::string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		std::cout << "contact force[" << name << "] = " << force.transpose() << std::endl;
	}
	std::cout << std::endl;


	// Computing the joint space inertia matrix
	Eigen::MatrixXd inertial_mat;
	wdyn.computeJointSpaceInertialMatrix(inertial_mat, ws.base_pos, ws.joint_pos);
	std::cout << "--------------- Joint Space Inertia Matrix --------------" << std::endl;
	std::cout << inertial_mat << " = inertial matrix" << std::endl;

	// Computing the centroidal inertia matrix
	Eigen::MatrixXd com_inertial_mat;
	wdyn.computeCentroidalInertialMatrix(com_inertial_mat, ws.base_pos, ws.joint_pos);
	std::cout << "--------------- _Centroidal Inertia Matrix --------------" << std::endl;
	std::cout << com_inertial_mat << " = centroidal inertial matrix" << std::endl;


	// Computing the instantaneous capture point
	Eigen::Vector3d icp_pos, com_pos, com_vel;
	com_pos = Eigen::Vector3d(0.2, 0.05, 0.85);
	com_vel = Eigen::Vector3d(0.1, 0.1, 0.);
	double height = 0.85;
	wdyn.computeInstantaneousCapturePoint(icp_pos, com_pos, com_vel, height);
	std::cout << "--------------- Instantaneous Capture Point --------------" << std::endl;
	std::cout << icp_pos.transpose() << " = icp" << std::endl;


	// Computing the centroidal moment pivot
	Eigen::Vector3d cmp_pos;
	wdyn.computeCentroidalMomentPivot(cmp_pos, com_pos, height, contact_forces);
	std::cout << "--------------- Centroidal Moment Pivot --------------" << std::endl;
	std::cout << cmp_pos.transpose() << " = cmp" << std::endl;

	return 0;
}
