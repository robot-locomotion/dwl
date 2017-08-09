#include <dwl/WholeBodyState.h>
#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <ctime>
#include <chrono>


int main(int argc, char **argv)
{
	// The number of iterations
	unsigned int N = 1000000;

	dwl::WholeBodyState ws;
	dwl::model::FloatingBaseSystem fbs;
	dwl::model::WholeBodyKinematics wkin;
	dwl::model::WholeBodyDynamics wdyn;

	// Resetting the system from the hyq urdf file
	std::string urdf_file = DWL_SOURCE_DIR"/sample/hyq.urdf";
	std::string yarf_file = DWL_SOURCE_DIR"/config/hyq.yarf";
	wdyn.modelFromURDFFile(urdf_file, yarf_file);
	wkin = wdyn.getWholeBodyKinematics();
	fbs = wdyn.getFloatingBaseSystem();

	// Define the DoF after initializing the robot model
	ws.setJointDoF(fbs.getJointDoF());

	wkin.setIKSolver( 1.0e-12, 0.01, 50);


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


	dwl::rbd::BodyVector6d grf;
	grf["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rh_foot"] << 0, 0, 0, 0, 0, 190.778;


	std::clock_t startcputime = std::clock();
	dwl::rbd::BodyVectorXd contact_pos_W;
	for (unsigned int i = 0; i < N; ++i)
		contact_pos_W = wkin.computePosition(ws.base_pos, ws.joint_pos,
											 fbs.getEndEffectorNames(), // it uses all the end-effector of the system
											 dwl::rbd::Linear, dwl::RollPitchYaw);

	double cpu_duration =
				(std::clock() - startcputime) * 1000000 / (double) CLOCKS_PER_SEC;
	std::cout << "  Forward kinematics: " << cpu_duration / N << " (microsecs, CPU time)" << std::endl;


	dwl::rbd::BodyVector3d ik_pos;
	ik_pos[fbs.getFloatingBaseName()] = Eigen::Vector3d::Zero();
	ik_pos["lf_foot"] = contact_pos_W.find("lf_foot")->second.tail(3);
	ik_pos["rf_foot"] = contact_pos_W.find("rf_foot")->second.tail(3);
	ik_pos["lh_foot"] = contact_pos_W.find("lh_foot")->second.tail(3);
	ik_pos["rh_foot"] = contact_pos_W.find("rh_foot")->second.tail(3);
	dwl::rbd::Vector6d base_pos_init = dwl::rbd::Vector6d::Zero();
	Eigen::VectorXd joint_pos_init(12);
	joint_pos_init << 0., 0.5, -1., 0., -0.5, 1., 0., 0.5, -1., 0., -0.5, 1.;
	startcputime = std::clock();
	for (unsigned int i = 0; i < N; ++i)
		wkin.computeInverseKinematics(ws.base_pos, ws.joint_pos,
									  ik_pos,
									  base_pos_init, joint_pos_init);

	cpu_duration =
				(std::clock() - startcputime) * 1000000 / (double) CLOCKS_PER_SEC;
	std::cout << "  Inverse kinematics: " << cpu_duration / N << " (microsecs, CPU time)" << std::endl;


	startcputime = std::clock();
	for (unsigned int i = 0; i < N; ++i)
		wdyn.computeInverseDynamics(ws.base_eff, ws.joint_eff,
									ws.base_pos, ws.joint_pos,
									ws.base_vel, ws.joint_vel,
									ws.base_acc, ws.joint_acc, grf);

	cpu_duration =
			(std::clock() - startcputime) * 1000000 / (double) CLOCKS_PER_SEC;
	std::cout << "  Inverse dynamics: " << cpu_duration / N << " (microsecs, CPU time)" << std::endl;


	Eigen::MatrixXd inertial_mat;
	startcputime = std::clock();
	for (unsigned int i = 0; i < N; ++i)
		wdyn.computeJointSpaceInertialMatrix(inertial_mat, ws.base_pos, ws.joint_pos);

	cpu_duration =
				(std::clock() - startcputime) * 1000000 / (double) CLOCKS_PER_SEC;
	std::cout << "  Joint space inertia matrix: " << cpu_duration / N << " (microsecs, CPU time)" << std::endl;
	return 0;
}
