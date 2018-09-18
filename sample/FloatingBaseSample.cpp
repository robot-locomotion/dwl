#include <dwl/model/FloatingBaseSystem.h>

using namespace std;



int main(int argc, char **argv)
{
	// Floating-base system object
	dwl::model::FloatingBaseSystem fbs;

	// Resetting the system from the hyq urdf file
	string model_file = DWL_SOURCE_DIR"/models/hyq.urdf";
	string robot_file = DWL_SOURCE_DIR"/models/hyq.yarf";
	fbs.resetFromURDFFile(model_file, robot_file);

	// Getting the total mass of the fbstem. Note that you could also get the mass of a specific
	// body (e.g. fbs.getBodyMass(body_name))
	cout << "total mass: " << fbs.getTotalMass() << endl;
	cout << "lf_upperleg mass: " << fbs.getBodyMass("lf_upperleg") << endl;
	cout << "gravity acceleration is " << fbs.getGravityAcceleration() << endl << endl;


	// Getting the CoM of the floating-base body. Note that you could also get the CoM of a
	// specific body (e.g. fbs.getBodyCoM(body_name))
	cout << "floating-base CoM: " << fbs.getFloatingBaseCoM().transpose() << endl;
	cout << "lf_upperleg CoM: " << fbs.getBodyCoM("lf_upperleg").transpose() << endl << endl;


	// Getting the number of system DoF, floating-base Dof, joints and end-effectors
	cout << "NQ: " << fbs.getConfigurationDim() << endl;
	cout << "NV: " << fbs.getTangentDim() << endl;
	cout << "number of joints: " << fbs.getJointDoF() << endl;
	cout << "number of end-effectors: " << fbs.getNumberOfEndEffectors() << endl;
	cout << "floating-base body name:" << fbs.getFloatingBaseName() << endl << endl;


	// Getting the joint names and ids
	for (dwl::model::ElementId::const_iterator it = fbs.getJoints().begin();
			it != fbs.getJoints().end(); ++it) {
		string name = it->first;
		unsigned int id = it->second;
		cout << "joint[" << id << "] = " << name << endl;
	}
	cout << endl;


	// Getting the body names and ids
	for (dwl::model::ElementId::const_iterator it = fbs.getBodies().begin();
			it != fbs.getBodies().end(); ++it) {
		string name = it->first;
		unsigned int id = it->second;
		cout << "body[" << id << "] = " << name << endl;
	}
	cout << endl;


	// Getting the frame names and ids
	for (dwl::model::ElementId::const_iterator it = fbs.getFrames().begin();
			it != fbs.getFrames().end(); ++it) {
		string name = it->first;
		unsigned int id = it->second;
		cout << "frame[" << id << "] = " << name << endl;
	}
	cout << endl;


	// Getting the end-effector names and ids
	dwl::model::ElementId contact_links = fbs.getEndEffectors();
	for (dwl::model::ElementId::const_iterator it = contact_links.begin();
			it != contact_links.end(); ++it) {
		string name = it->first;
		unsigned int id = it->second;

		cout << "end-effector[" << id << "] = " << name << endl;
	}
	cout << endl;


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


	// Getting the default posture
	cout << "'nominal joint positions = " << fbs.getDefaultPosture().transpose() << endl << endl;


	// Getting the CoM of the floating-base body. Note that you could also get the CoM of a
	// specific body (e.g. fbs.getBodyCoM(body_name))
	cout << "Floating-base inertia: " << endl << fbs.getFloatingBaseInertia() << endl;
	cout << "lf_hipassembly inertia: " << endl << fbs.getBodyInertia("lf_hipassembly") << endl << endl;


	// Conversion between whole-body state to generalized state, and viceverse
	dwl::SE3 base_pos;
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(fbs.getJointDoF());
	Eigen::VectorXd generalized_state =
			fbs.toConfigurationState(base_pos, joint_pos);
	cout << "Converting the whole-body pose to a configuration vector" << endl;
	cout << "base position = " << base_pos.data << endl;
	cout << "joint position =" << joint_pos.transpose() << endl;
	cout << "configuration vector = " << generalized_state.transpose() << endl << endl;


	// Setting up the branch states
	Eigen::Vector3d lf_branch_pos = Eigen::Vector3d(0.5, 0.75, 1.5);
	fbs.setBranchState(joint_pos, lf_branch_pos, "lf_foot");
	cout << "Setting up lf_foot branch position = " << lf_branch_pos.transpose() << endl;
	cout << "joint position = " << joint_pos.transpose() << endl;
	cout << "Getting the lf_foot branch position = ";
	cout << fbs.getBranchState(joint_pos, "lf_foot").transpose() << endl << endl;

	return 0;
}
