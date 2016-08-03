#include <dwl/model/WholeBodyDynamics.h>

using namespace std;



int main(int argc, char **argv)
{
	// Floating-base system & Whole-body dynamics object
	dwl::model::FloatingBaseSystem sys;
	dwl::model::WholeBodyDynamics dyn;

	// Resetting the system from the hyq urdf file
	string model_file = "../sample/hyq.urdf";
	string robot_file = "../config/hyq.yarf";
	sys.resetFromURDFFile(model_file, robot_file);
	dyn.modelFromURDFFile(model_file, robot_file);


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


	// Computing the ID
	dwl::rbd::BodyVector6d grf;
	grf["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rh_foot"] << 0, 0, 0, 0, 0, 190.778;
	dyn.computeInverseDynamics(base_wrench, joint_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc, grf);
	cout << "--------------------------- ID --------------------------" << endl;
	cout << "Base wrench = " << base_wrench.transpose() << endl;
	cout << "Joint forces = " << joint_forces.transpose() << endl << endl;


	// Estimating active contacts
	double force_threshold = 0.;
	dwl::rbd::BodySelector active_contacts;
	dwl::rbd::BodyVector6d contact_forces;
	dyn.estimateActiveContacts(active_contacts, contact_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc,
							   joint_forces, sys.getEndEffectorNames(), // it uses all the end-effector of the system
							   force_threshold);
	cout << "--------------- Estimated active contacts ---------------" << endl;
	for (dwl::rbd::BodyVector6d::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		cout << "active contact: " << name << " and its force is " << force.transpose() << endl << endl;
	}


	// Estimating contact forces from joint measurements
	dyn.computeContactForces(contact_forces, joint_forces,
							 base_pos, joint_pos,
							 base_vel, joint_vel,
							 base_acc, joint_acc,
							 sys.getEndEffectorNames());
	cout << "--------------- Estimated contact forces ----------------" << endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;
	for (dwl::rbd::BodyVector6d::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		cout << "contact force[" << name << "] = " << force.transpose() << endl;
	}
	cout << endl;


	// Computing the floating-base ID
	dyn.computeFloatingBaseInverseDynamics(base_acc, joint_forces,
										   base_pos, joint_pos,
										   base_vel, joint_vel,
										   joint_acc, contact_forces);
	cout << "---------------- Floating-base ID ------------------" << std::endl;
	cout << "Base accelerations = " << base_acc.transpose() << endl;
	cout << "Joint forces = " << joint_forces.transpose() << endl << endl;


	// Computing the constrained ID
	base_acc.setZero();
	dyn.computeConstrainedFloatingBaseInverseDynamics(joint_forces,
													  base_pos, joint_pos,
													  base_vel, joint_vel,
													  base_acc, joint_acc,
													  sys.getEndEffectorNames());
	cout << "------------------ Constrained ID ------------------" << std::endl;
	cout << "Joint forces = " << joint_forces.transpose() << endl << endl;


	// Computing the contact forces from the CoP
	Eigen::Vector3d cop_pos(0.0196, -0.0012, 0.0215);
	dwl::rbd::BodyVector3d contact_pos;
	contact_pos["lf_foot"] = Eigen::Vector3d(0.319215579664, 0.206424153349, 0.0215);
	contact_pos["lh_foot"] = Eigen::Vector3d(-0.335953242968, 0.207404146377, 0.0215);
	contact_pos["rf_foot"] = Eigen::Vector3d(0.31996232038, -0.207592286639, 0.0215);
	contact_pos["rh_foot"] = Eigen::Vector3d(-0.331894998575, -0.207236136594, 0.0215);
	dyn.computeContactForces(contact_forces,
							 cop_pos,
							 contact_pos,
							 sys.getEndEffectorNames());
	cout << "----------------- Contact forces from CoP ---------------" << endl;
	for (dwl::rbd::BodyVector6d::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		cout << "contact force[" << name << "] = " << force.transpose() << endl;
	}
	cout << endl;


	// Computing the joint space inertia matrix
	Eigen::MatrixXd inertial_mat;
	dyn.computeJointSpaceInertialMatrix(inertial_mat, base_pos, joint_pos);
	cout << "--------------- Joint Space Inertia Matrix --------------" << endl;
	cout << inertial_mat << " = inertial matrix" << endl;

	return 0;
}
