#include <dwl/model/FloatingBaseSystem.h>

using namespace std;



int main(int argc, char **argv)
{
	// Floating-base system object
	dwl::model::FloatingBaseSystem sys;

	// Resetting the system from the hyq urdf file
	string model_file = "../sample/hyq.urdf";
	string robot_file = "../config/hyq.yarf";
	sys.resetFromURDFFile(model_file, robot_file);

	// Getting the total mass of the system. Note that you could also get the mass of a specific
	// body (e.g. sys.getBodyMass(body_name))
	cout << "Total mass: " << sys.getTotalMass() << endl;

	// Getting the CoM of the floating-base body. Note that you could also get the CoM of a
	// specific body (e.g. sys.getBodyCoM(body_name))
	cout << "Floating-base CoM: " << sys.getFloatingBaseCoM().transpose() << endl << endl;

	// Getting the number of system DoF, floating-base Dof, joints and end-effectors
	cout << "Total dof: " << sys.getSystemDoF() << endl;
	cout << "Floating-base dof: " << sys.getFloatingBaseDoF() << endl;
	cout << "Number of joint: " << sys.getJointDoF() << endl;
	cout << "Number of end-effectors: " << sys.getNumberOfEndEffectors() << endl << endl;

	// Getting the floating-base information
	for (unsigned int i = 0; i < 6; i++) {
		dwl::rbd::Coords6d base_id = dwl::rbd::Coords6d(i);
		dwl::model::FloatingBaseJoint jnt = sys.getFloatingBaseJoint(base_id);

		if (jnt.active) {
			cout << "Base joint[" << jnt.id << "] = " << jnt.name << endl;
		}
	}
	cout << endl;

	// Getting the joint names and ids
	dwl::urdf_model::JointID joint_links = sys.getJoints();
	for (dwl::urdf_model::JointID::const_iterator joint_it = joint_links.begin();
			joint_it != joint_links.end(); ++joint_it) {
		string name = joint_it->first;
		unsigned int id = joint_it->second;

		cout << "Joint[" << id << "] = " << name << endl;
	}
	cout << endl;

	// Getting the end-effector names and ids
	dwl::urdf_model::LinkID contact_links = sys.getEndEffectors();
	for (dwl::urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
			contact_it != contact_links.end(); contact_it++) {
		string name = contact_it->first;
		unsigned int id = contact_it->second;

		cout << "End-effector[" << id << "] = " << name << endl;
	}
	cout << endl;

	// Getting the default posture
	Eigen::VectorXd joint_pos0 = sys.getDefaultPosture();

	// Getting the joint limits
	dwl::urdf_model::JointLimits joint_limits = sys.getJointLimits();
	dwl::urdf_model::JointID ids = sys.getJoints();
	for (dwl::urdf_model::JointLimits::iterator jnt_it = joint_limits.begin();
			jnt_it != joint_limits.end(); ++jnt_it) {
		string name = jnt_it->first;
		urdf::JointLimits limits = jnt_it->second;
		unsigned int id = sys.getJointId(name);

		cout << name << "[" << id << "].lower = " << limits.lower << endl;
		cout << name << "[" << id << "].upper = " << limits.upper << endl;
		cout << name << "[" << id << "].velocity = " << limits.velocity << endl;
		cout << name << "[" << id << "].effort = " << limits.effort << endl;
		cout << name << "[" << id << "].pos0 = " << joint_pos0(id) << endl << endl;
	}

	// Setting up the branch states
	dwl::rbd::Vector6d base_pos = dwl::rbd::Vector6d::Zero();
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(sys.getJointDoF());
	Eigen::Vector3d lf_branch_pos = Eigen::Vector3d(0.5, 0.75, 1.5);
	sys.setBranchState(joint_pos, lf_branch_pos, "lf_foot");
	cout << "Setting up lf_foot branch position = " << lf_branch_pos.transpose() << endl;
	cout << "Base position = " << base_pos.transpose() << endl;
	cout << "Joint position = " << joint_pos.transpose() << endl;
	cout << "Getting the lf_foot branch position = ";
	cout << sys.getBranchState(joint_pos, "lf_foot").transpose() << endl << endl;

	return 0;
}
