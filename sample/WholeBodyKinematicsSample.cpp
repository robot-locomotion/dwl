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

	ws.setJointAcceleration(1., fbs.getJointId("lf_hfe_joint"));

	
	// Computing the CoM position and velocity
	Eigen::Vector3d x, xd;
	wkin.computeCoMRate(x, xd,
						ws.base_pos, ws.joint_pos,
						ws.base_vel, ws.joint_vel);
	cout << "x = " << x.transpose() << endl;
	cout << "xd = " << xd.transpose() << endl << endl;

	// Computing the Cartesian position of a set of frames
	Eigen::Vector7dMap pos_W =
		wkin.computePosition(ws.base_pos, ws.joint_pos,
							 fbs.getEndEffectorNames());
	cout << "Frame position:" << endl;
	for (Eigen::Vector7dMap::iterator it = pos_W.begin();
		it != pos_W.end(); ++it)
			cout << "  " << it->first << " = " << it->second.transpose() << endl;
	cout << endl;
	// ws.setContactPosition_W(pos_W);


	// Computing the Cartesian velocity of a set of frames. A frame is a fixed-point
	// in a body
	Eigen::Vector6dMap vel_W = 
		wkin.computeVelocity(ws.base_pos, ws.joint_pos,
							 ws.base_vel, ws.joint_vel,
							 fbs.getEndEffectorNames());
	cout << "Frame velocity:" << endl;
	for (Eigen::Vector6dMap::iterator it = vel_W.begin();
		it != vel_W.end(); ++it)
			cout << "  " << it->first << " = " << it->second.transpose() << endl;
	cout << endl;
	// ws.setContactVelocity_W(pos_W);
	

	// Computing the Cartesian acceleration of a set of frames
	Eigen::Vector6dMap acc_W = 
		wkin.computeAcceleration(ws.base_pos, ws.joint_pos,
								 ws.base_vel, ws.joint_vel,
								 ws.base_acc, ws.joint_acc,
								 fbs.getEndEffectorNames());
	cout << "Frame acceleration:" << endl;
	for (Eigen::Vector6dMap::iterator it = acc_W.begin();
		it != acc_W.end(); ++it)
			cout << "  " << it->first << " = " << it->second.transpose() << endl;
	cout << endl;
	// ws.setContactAcceleration_W(pos_W);
	
	
	// Computing the contact Jacd*Qd
	Eigen::Vector6dMap jd_qd_W =
		wkin.computeJdQd(ws.base_pos, ws.joint_pos,
						 ws.base_vel, ws.joint_vel,
						 fbs.getEndEffectorNames());
	cout << "Frame Jd*qd term:" << endl;
	for (Eigen::Vector6dMap::iterator it = jd_qd_W.begin();
			it != jd_qd_W.end(); ++it)
		cout << "  " << it->first << " = " << it->second.transpose() << endl;
	cout << endl;

	// Computing the jacobians
	Eigen::MatrixXd jacobian, fixed_jac, floating_jac;
	Eigen::Matrix6xMap jac = 
		wkin.computeJacobian(ws.base_pos, ws.joint_pos,
							 fbs.getEndEffectorNames());
	cout << "Frame Jacobian:" << endl;
	for (Eigen::Matrix6xMap::iterator it = jac.begin();
		it != jac.end(); ++it) {
			cout << it->first << ":" << endl;
			cout << it->second << endl;
			cout << "---" << endl;
			Eigen::Matrix6d floating_jac;
			Eigen::Matrix6x fixed_jac;
			wkin.getFloatingBaseJacobian(floating_jac, it->second);
			wkin.getFixedBaseJacobian(fixed_jac, it->second);
			cout << floating_jac << " = Floating-base Jacobian" << endl;
			cout << "---" << endl;
			cout << fixed_jac << " = Fixed-base Jacobian" << endl;
			cout << endl;
	}
	cout << endl;
	cout << "---------------------------------------" << endl;



	// // Computing IK
	Eigen::Vector7dMap frame_pos;// = pos_W;
	frame_pos["lf_foot"] = pos_W.find("lf_foot")->second;
	frame_pos["lh_foot"] = pos_W.find("lh_foot")->second;
	// wkin.setIKSolver( 1.0e-12, 0.01, 50);
	Eigen::VectorXd joint_pos0(12);
	joint_pos0 << 0., 0.5, -1., 0., -0.5, 1., 0., 0.5, -1., 0., -0.5, 1.;
	if (wkin.computeJointPosition(ws.joint_pos,
								  frame_pos,
								  joint_pos0)) {
		cout << "Joint position = "<< ws.joint_pos.transpose() << endl << endl;
	} else {
		cout << "The IK problem could not be solved" << endl;
	}

	Eigen::Vector6dMap frame_vel;
	frame_vel["lf_foot"] << 0., 0., 0., 1., 0., 0.;
	frame_vel["lh_foot"] << 0., 0., 0., 0., 1., 0.;
	frame_vel["rf_foot"] << 0., 0., 0., 0., 0., 1.;
	wkin.computeJointVelocity(ws.joint_vel, ws.joint_pos, frame_vel);
	cout << "Joint velocity = " << ws.joint_vel.transpose() << endl << endl;


	Eigen::Vector6dMap frame_acc;
	frame_acc["lf_foot"] << 0., 0., 0., 1., 0., 0.;
	frame_acc["lh_foot"] << 0., 0., 0., 0., 1., 0.;
	frame_acc["rf_foot"] << 0., 0., 0., 0., 0., 1.;
	wkin.computeJointAcceleration(ws.joint_acc, ws.joint_pos, 0.*ws.joint_vel, frame_acc);
	cout << "Joint acceleration = " << ws.joint_acc.transpose() << endl << endl;


    return 0;
}
