#include <dwl/WholeBodyState.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/utils/macros.h>



int main(int argc, char **argv)
{
	// Resetting the system from the hyq urdf file
	std::string model_file = "../sample/hyq.urdf";
	std::string robot_file = "../config/hyq.yarf";
	dwl::model::FloatingBaseSystem fbs;
	fbs.resetFromURDFFile(model_file, robot_file);


	dwl::WholeBodyState ws(fbs.getJointDoF());
	ws.setBasePosition_W(Eigen::Vector3d(0.2, 0.1, 0.56));
	ws.setBaseRPY_W(Eigen::Vector3d(0., 0., 0.5));



	PRINT_VECTOR(ws.getBasePosition_W());
	PRINT_VECTOR(ws.getBaseRPY_W());
	PRINT_VECTOR(ws.getBaseOrientation_W().coeffs());
	PRINT_VECTOR(ws.getBaseOrientation_H().coeffs());
	PRINT_VECTOR(ws.getBaseOrientation_H().coeffs());

	ws.setJointPosition(0.3, 2);
	ws.setJointVelocity(1., 5);
	ws.setJointAcceleration(0.5, 11);
	ws.setJointEffort(50., 0);
	PRINT_VECTOR(ws.getJointPosition());

	ws.setContactPosition_B("lf_foot", Eigen::Vector3d(0.2, 0.5, -0.5));
	PRINT_VECTOR(ws.getContactPosition_B("lf_foot"));


	std::cout << "base_pos = " << ws.base_pos.transpose() << std::endl;
	std::cout << "base_vel = " << ws.base_vel.transpose() << std::endl;
	std::cout << "base_acc = " << ws.base_acc.transpose() << std::endl;
	std::cout << "joint_pos = " << ws.joint_pos.transpose() << std::endl;
	std::cout << "joint_vel = " << ws.joint_vel.transpose() << std::endl;
	std::cout << "joint_acc = " << ws.joint_acc.transpose() << std::endl;
	std::cout << "joint_eff = " << ws.joint_eff.transpose() << std::endl;

	return 0;
}
