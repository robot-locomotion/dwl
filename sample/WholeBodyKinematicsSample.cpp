#include <dwl/WholeBodyState.h>
#include <dwl/model/WholeBodyKinematics.h>


using namespace std;

int main(int argc, char **argv)
{
	dwl::WholeBodyState ws;
	dwl::model::FloatingBaseSystem fbs;
	dwl::model::WholeBodyKinematics wkin;

	// Resetting the system from the hyq urdf file
	std::string urdf_file = DWL_SOURCE_DIR"/models/hyq.urdf";
	std::string yarf_file = DWL_SOURCE_DIR"/models/hyq.yarf";
	fbs.resetFromURDFFile(urdf_file, yarf_file);
	wkin.reset(fbs);

	// Define the DoF after initializing the robot model
	ws.setJointDoF(fbs.getJointDoF());


	// The robot state
	ws.setBaseSE3(dwl::SE3(Eigen::Vector3d(0., 0., 0.),
						   Eigen::Vector3d(0., 0., 0.)));
	ws.setBaseVelocity_W(dwl::Motion(Eigen::Vector3d(0., 0., 0.),
									 Eigen::Vector3d(0., 0., 0.)));
	ws.setBaseAcceleration_W(dwl::Motion(Eigen::Vector3d(0., 0., 0.),
										 Eigen::Vector3d(0., 0., 0.)));
	ws.setJointPosition(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointPosition(-1.5, fbs.getJointId("lf_kfe_joint"));
	ws.setJointPosition(-0.75, fbs.getJointId("lh_hfe_joint"));
	ws.setJointPosition(1.5, fbs.getJointId("lh_kfe_joint"));
	ws.setJointPosition(0.75, fbs.getJointId("rf_hfe_joint"));
	ws.setJointPosition(-1.5, fbs.getJointId("rf_kfe_joint"));
	ws.setJointPosition(-0.75, fbs.getJointId("rh_hfe_joint"));
	ws.setJointPosition(1.5, fbs.getJointId("rh_kfe_joint"));

	ws.setJointVelocity(0.2, fbs.getJointId("lf_haa_joint"));
	ws.setJointVelocity(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointVelocity(1., fbs.getJointId("lf_kfe_joint"));

	ws.setJointAcceleration(0.2, fbs.getJointId("lf_haa_joint"));
	ws.setJointAcceleration(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointAcceleration(1., fbs.getJointId("lf_kfe_joint"));

	
	// Computing the Jacobians
	Eigen::Matrix6xMap jacobian =
		wkin.computeJacobian(ws.getBaseSE3(), ws.getJointPosition(),
							 fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Stack of Jacobians:" << endl;
	for (Eigen::Matrix6xMap::iterator it = jacobian.begin();
		it != jacobian.end(); ++it) {
			cout << it->first << ":" << endl;
			cout << it->second << endl;
			cout << "---" << endl;
	}
	cout << endl << endl;


	// Getting the floating-base jacobian (LF foot)
	Eigen::Matrix6d floating_jac;
	wkin.getFloatingBaseJacobian(floating_jac, jacobian["lf_foot"]);
	cout << "Floating-base Jacobian (LF foot):" << endl;
	cout << floating_jac << endl;


	// Getting the fixed-base jacobian (LF foot)
	Eigen::Matrix6x fixed_jac;
	wkin.getFixedBaseJacobian(fixed_jac, jacobian["lf_foot"]);
	cout << "Fixed-base Jacobian (LF foot):" << endl;
	cout << fixed_jac << endl << endl;


	// Computing the Cartesian position of a set of frames
	dwl::SE3Map contact_pos_W =
		wkin.computePosition(ws.getBaseSE3(), ws.getJointPosition(),
							 fbs.getEndEffectorList());
	cout << "Contact SE3:" << endl;
	for (dwl::SE3Map::iterator it = contact_pos_W.begin();
		it != contact_pos_W.end(); ++it) {
			cout << it->first << ":" << endl;
			cout << it->second.data << endl;
	}
	cout << endl;
	// ws.setContactPosition_W(pos_W);


	// Computing the Cartesian velocity of a set of frames. A frame is a fixed-point
	// in a body
	dwl::MotionMap contact_vel_W =
		wkin.computeVelocity(ws.getBaseSE3(), ws.getJointPosition(),
							 ws.getBaseVelocity_W(), ws.getJointVelocity(),
							 fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Contact velocity:" << endl;
	for (dwl::MotionMap::iterator it = contact_vel_W.begin();
		it != contact_vel_W.end(); ++it) {
			cout << it->first << ":" << endl;
			cout << it->second.data << endl;
	}
	cout << endl;
	// ws.setContactVelocity_W(pos_W);


	// Computing the Cartesian acceleration of a set of frames
	dwl::MotionMap contact_acc_W =
		wkin.computeAcceleration(ws.getBaseSE3(), ws.getJointPosition(),
								 ws.getBaseVelocity_W(), ws.getJointVelocity(),
								 ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
								 fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Contact acceleration:" << endl;
	for (dwl::MotionMap::iterator it = contact_acc_W.begin();
		it != contact_acc_W.end(); ++it) {
			cout << it->first << ":" << endl;
			cout << it->second.data << endl;
	}
	cout << endl;
	// ws.setContactAcceleration_W(pos_W);


	// Computing the contact Jacd*Qd
	dwl::MotionMap contact_jdqd_W =
		wkin.computeJdQd(ws.getBaseSE3(), ws.getJointPosition(),
						 ws.getBaseVelocity_W(), ws.getJointVelocity(),
						 fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Contact Jd*qd term:" << endl;
	for (dwl::MotionMap::iterator it = contact_jdqd_W.begin();
		it != contact_jdqd_W.end(); ++it) {
			cout << it->first << ":" << endl;
			cout << it->second.data << endl;
	}
	cout << endl;



	// Computing the joint positions
	wkin.setIKSolver(1.0e-12, 50);
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(fbs.getJointDoF());
	dwl::SE3Map contact_pos_B;
	contact_pos_B["lf_foot"] =
			dwl::SE3(Eigen::Vector3d(0.371, 0.207, -0.589),
					(Eigen::Matrix3d) Eigen::Matrix3d::Identity());
	contact_pos_B["lh_foot"] =
			dwl::SE3(Eigen::Vector3d(0.371, 0.207, -0.589),
					(Eigen::Matrix3d) Eigen::Matrix3d::Identity());
	Eigen::VectorXd joint_pos_init = fbs.getDefaultPosture();
	if (wkin.computeJointPosition(joint_pos,
								  contact_pos_B,
								  joint_pos_init)) {
		cout << "Joint position = "<< ws.joint_pos.transpose() << endl << endl;
	} else {
		cout << "The IK problem could not be solved" << endl;
	}


	// Computing the joint velocities. Note that you can create and dwl::MotioMap()
	// object, similar as before
	Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(fbs.getJointDoF());
	dwl::MotionMap contact_vel_B;
	contact_vel_B["lf_foot"] = dwl::Motion(Eigen::Vector3d::Zero(),
									   Eigen::Vector3d(1., 0., 0.));
	contact_vel_B["lh_foot"] = dwl::Motion(Eigen::Vector3d::Zero(),
									   Eigen::Vector3d(0., 1., 0.));
	contact_vel_B["rf_foot"] = dwl::Motion(Eigen::Vector3d::Zero(),
									   Eigen::Vector3d(0., 0., 1.));
	wkin.computeJointVelocity(joint_vel,
							  ws.getJointPosition(),
							  contact_vel_B);
	cout << "Joint velocity = " << joint_vel.transpose() << endl << endl;


	// Computing the joint accelerations
	Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(fbs.getJointDoF());
	dwl::MotionMap contact_acc_B = contact_acc_W;
	wkin.computeJointAcceleration(joint_acc,
								  ws.getJointPosition(),
								  ws.getJointVelocity(),
								  contact_acc_B);
	cout << "Joint acceleration = " << joint_acc.transpose() << endl << endl;


	// Computing the CoM position, velocity and acceleration
	cout << "CoM position = " <<
			wkin.computeCoM(ws.getBaseSE3(),
							ws.getJointPosition()).transpose() << endl << endl;


	// Computing the CoM position and velocity
	Eigen::Vector3d x, xd, xdd;
	wkin.computeCoMRate(x, xd, xdd,
						ws.getBaseSE3(), ws.getJointPosition(),
						ws.getBaseVelocity_W(), ws.getJointVelocity(),
						ws.getBaseAcceleration_W(), ws.getJointAcceleration());
	cout << "CoM velocity = " << x.transpose() << endl << endl;
	cout << "CoM acceleration = " << xd.transpose() << endl << endl;


	// Computing the constrained acceleration
	ws.setBaseAcceleration_W(
			dwl::Motion(Eigen::Vector3d(1., 0., 0.),
						Eigen::Vector3d::Zero()));
	Eigen::VectorXd joint_facc = Eigen::VectorXd::Zero(fbs.getJointDoF());
	wkin.computeConstrainedJointAcceleration(joint_facc,
	                                         ws.getBaseSE3(), ws.getJointPosition(),
	                                         ws.getBaseVelocity_W(), ws.getJointVelocity(),
	                                         ws.getBaseAcceleration_W(),
	                                         fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "joint acceleration = " << joint_facc.transpose() << endl;


    return 0;
}
