#include <dwl/model/FloatingBaseSystem.h>

using namespace std;



int main(int argc, char **argv)
{
	// Floating-base system object
	dwl::model::FloatingBaseSystem fbs;

	// Resetting the system from the hyq urdf file
	string model_file = DWL_SOURCE_DIR"/sample/hyq.urdf";
	string robot_file = DWL_SOURCE_DIR"/config/hyq.yarf";
	fbs.resetFromURDFFile(model_file, robot_file);

	// Getting the number of system DoF, floating-base Dof, joints and end-effectors
	cout << "NQ: " << fbs.getConfigurationDim() << endl;
	cout << "NV: " << fbs.getTangentDim() << endl;
	// cout << "Floating-base dof: " << fbs.getFloatingBaseDoF() << endl;
	cout << "Number of joint: " << fbs.getJointDoF() << endl;
	cout << "Number of end-effectors: " << fbs.getNumberOfEndEffectors() << endl << endl;

	// Getting floating base name
	cout << "Floating-base name: " << fbs.getFloatingBaseName() << endl << endl;

	// Getting the joint names and ids
	for (unsigned int j = 0; j < fbs.getJointDoF(); ++j) {
		string name = fbs.getJointName(j);
		cout << "joint[" << j << "] = " << name << endl;
	}
	cout << endl;

	// Getting the total mass of the fbstem. Note that you could also get the mass of a specific
	// body (e.g. fbs.getBodyMass(body_name))
	cout << "Total mass: " << fbs.getTotalMass() << endl;
	cout << "trunk mass: " << fbs.getBodyMass("trunk") << endl << endl;

	// Getting the CoM of the floating-base body. Note that you could also get the CoM of a
	// specific body (e.g. fbs.getBodyCoM(body_name))
	cout << "Floating-base CoM: " << fbs.getFloatingBaseCoM().transpose() << endl;
	cout << "lf_hipassembly CoM: " << fbs.getBodyCoM("lf_hipassembly").transpose() << endl << endl;

	// Getting the CoM of the floating-base body. Note that you could also get the CoM of a
	// specific body (e.g. fbs.getBodyCoM(body_name))
	cout << "Floating-base inertia: " << endl << fbs.getFloatingBaseInertia() << endl;
	cout << "lf_hipassembly inertia: " << endl << fbs.getBodyInertia("lf_hipassembly") << endl << endl;


	// Getting the floating-base information
	// for (unsigned int i = 0; i < 6; i++) {
	// 	dwl::rbd::Coords6d base_id = dwl::rbd::Coords6d(i);
	// 	dwl::model::FloatingBaseJoint jnt = fbs.getFloatingBaseJoint(base_id);

	// 	if (jnt.active) {
	// 		cout << "Base joint[" << jnt.id << "] = " << jnt.name << endl;
	// 	}
	// }
	// cout << endl;


	// Getting the default posture
	cout << "default posture = " << fbs.getDefaultPosture().transpose() << endl << endl;

	// Getting the joint limits
	cout << "lower limits = " << fbs.getLowerLimits().transpose() << endl;
	cout << "upper limits = " << fbs.getUpperLimits().transpose() << endl;
	cout << "velocity limits = " << fbs.getVelocityLimits().transpose() << endl;
	cout << "effort limits = " << fbs.getEffortLimits().transpose() << endl << endl;
	// You can also get the limits given the name of the joint, i.e.
	// for (unsigned int j = 0; j < fbs.getJointDoF(); ++j) {
	// 	string name = fbs.getJointName(j);
	// 	cout << name << endl;
	// 	cout << "  lower = " << fbs.getLowerLimit(name) << endl;
	// 	cout << "  upper = " << fbs.getUpperLimit(name) << endl;
	// 	cout << "  vel = " << fbs.getVelocityLimit(name) << endl;
	// 	cout << "  eff = " << fbs.getEffortLimit(name) << endl;
	// }

	// Getting the end-effector names and ids
	dwl::model::ElementId contact_links = fbs.getEndEffectors();
	for (dwl::model::ElementId::const_iterator it = contact_links.begin();
			it != contact_links.end(); it++) {
		string name = it->first;
		unsigned int id = it->second;

		cout << "end-effector[" << id << "] = " << name << endl;
	}
	cout << endl;

	// Getting gravity vector and acceleration
	cout << "g = " << fbs.getGravityVector().transpose() << endl;
	cout << "g_acc = " << fbs.getGravityAcceleration() << endl << endl;

	// Setting up the branch states
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(fbs.getJointDoF());
	Eigen::Vector3d lf_branch_pos = Eigen::Vector3d(0.5, 0.75, 1.5);
	fbs.setBranchState(joint_pos, lf_branch_pos, "lf_foot");
	cout << "Setting up lf_foot branch position = " << lf_branch_pos.transpose() << endl;
	cout << "Joint position = " << joint_pos.transpose() << endl;
	cout << "Getting the lf_foot branch position = " << fbs.getBranchState(joint_pos, "lf_foot").transpose() << endl << endl;

	return 0;
}
