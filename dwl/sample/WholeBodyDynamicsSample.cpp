#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/model/WholeBodyDynamics.h>

using namespace std;



int main(int argc, char **argv)
{
	// Floating-base system & Whole-body dynamics object
	dwl::model::FloatingBaseSystem sys;
	dwl::model::WholeBodyDynamics dyn;

	// Resetting the system from the hyq urdf file
	string model_file = "../sample/hyq.urdf";
	string system_file = "../config/system_config.yaml";
	sys.resetFromURDFFile(model_file, system_file);
	dyn.modelFromURDFFile(model_file, system_file);


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
	dwl::rbd::BodyWrench grf;
	grf["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rh_foot"] << 0, 0, 0, 0, 0, 190.778;
	dyn.computeInverseDynamics(base_wrench, joint_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc, grf);
	cout << "------------------ ID ---------------------" << endl;
	cout << "Base wrench = " << base_wrench.transpose() << endl;
	cout << "Joint forces = " << joint_forces.transpose() << endl << endl;


	// Estimating active contacts
	double force_threshold = 0.;
	dwl::rbd::BodySelector active_contacts;
	dwl::rbd::BodyWrench contact_forces;
	dyn.estimateActiveContacts(active_contacts, contact_forces,
							   base_pos, joint_pos,
							   base_vel, joint_vel,
							   base_acc, joint_acc,
							   joint_forces, sys.getEndEffectorNames(), // it uses all the end-effector of the system
							   force_threshold);
	cout << "-------------------- Estimated active contacts -------------------" << endl;
	for (dwl::rbd::BodyWrench::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		cout << "active contact: " << name << " and its force is " << force.transpose() << endl;
	}


	// Estimating contact forces from joint measurements
	dyn.computeContactForces(contact_forces, joint_forces,
							 base_pos, joint_pos,
							 base_vel, joint_vel,
							 base_acc, joint_acc,
							 sys.getEndEffectorNames());
	cout << "-------------------- Estimated contact forces -------------------" << endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;
	for (dwl::rbd::BodyWrench::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		string name = it->first;
		dwl::rbd::Vector6d force = it->second;

		cout << "contact force[" << name << "] = " << force.transpose() << endl;
	}


	// Computing the floating-base ID
	dyn.computeFloatingBaseInverseDynamics(base_acc, joint_forces,
										   base_pos, joint_pos,
										   base_vel, joint_vel,
										   joint_acc, contact_forces);
	cout << "------------------- Floating-base ID --------------------" << std::endl;
	cout << "Base accelerations = " << base_acc.transpose() << endl;
	cout << "Joint forces = " << joint_forces.transpose() << endl << endl;


	// Computing the constrained ID
	base_acc.setZero();
	dyn.computeConstrainedFloatingBaseInverseDynamics(joint_forces,
													  base_pos, joint_pos,
													  base_vel, joint_vel,
													  base_acc, joint_acc,
													  sys.getEndEffectorNames());
	std::cout << "------------------- Constrained ID --------------------" << std::endl;
	std::cout << "Joint forces = " << joint_forces.transpose() << std::endl;


	return 0;
}
