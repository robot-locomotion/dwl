#include <dwl/WholeBodyState.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <ctime>
#include <chrono>


int main(int argc, char **argv)
{
	// The number of iterations
	unsigned int N = 1000000;

	dwl::WholeBodyState ws;
	dwl::model::FloatingBaseSystem fbs;
	dwl::model::WholeBodyDynamics wdyn;

	// Resetting the system from the hyq urdf file
	std::string urdf_file = DWL_SOURCE_DIR"/sample/hyq.urdf";
	std::string yarf_file = DWL_SOURCE_DIR"/config/hyq.yarf";
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

	dwl::rbd::BodyVector6d grf;
	grf["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
	grf["rh_foot"] << 0, 0, 0, 0, 0, 190.778;


	std::clock_t startcputime = std::clock();
	for (unsigned int i = 0; i < N; ++i)
		wdyn.computeInverseDynamics(ws.base_eff, ws.joint_eff,
									ws.base_pos, ws.joint_pos,
									ws.base_vel, ws.joint_vel,
									ws.base_acc, ws.joint_acc, grf);

	double cpu_duration =
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
