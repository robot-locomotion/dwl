#include <dwl/WholeBodyState.h>

#define BOOST_TEST_MODULE DWL_TESTS
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>



// Tolerance
double epsilon = 0.00001;

BOOST_AUTO_TEST_CASE(base_linear) // specify a test case for base linear states
{
	dwl::WholeBodyState ws;
	Eigen::Vector3d old_base_state;
	Eigen::Vector3d new_base_state;

	// Testing the base position
	old_base_state = Eigen::Vector3d(0.1, 0.2, 0.6);
	ws.setBasePosition_W(old_base_state);
	new_base_state = ws.getBasePosition_W();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);


	// Testing the base velocity
	old_base_state = Eigen::Vector3d(0.5, 0.7, 0.);
	ws.setBaseVelocity_W(old_base_state);
	new_base_state = ws.getBaseVelocity_W();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseVelocity_B();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseVelocity_H();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);


	// Testing the base acceleration
	old_base_state = Eigen::Vector3d(0.8, 0.9, -9.8);
	ws.setBaseAcceleration_W(old_base_state);
	new_base_state = ws.getBaseAcceleration_W();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseAcceleration_B();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseAcceleration_H();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);
}


BOOST_AUTO_TEST_CASE(base_angular) // specify a test case for base angular states
{
	dwl::WholeBodyState ws;
	Eigen::Vector3d old_base_state;
	Eigen::Vector3d new_base_state;
	Eigen::Vector3d rpy;
	Eigen::Quaterniond q;

	// Testing RPY
	ws.setBaseRPY_W(Eigen::Vector3d(0., M_PI_4, M_PI_4));
	ws.setBaseOrientation_W(ws.getBaseOrientation_W());
	rpy = ws.getBaseRPY_W();
	BOOST_CHECK_SMALL((double) rpy(0) - 0., epsilon);
	BOOST_CHECK_SMALL((double) rpy(1) - M_PI_4, epsilon);
	BOOST_CHECK_SMALL((double) rpy(2) - M_PI_4, epsilon);


	// Testing quaternion in the world frame
	ws.setBaseRPY_W(Eigen::Vector3d(0., 0., M_PI));
	q = ws.getBaseOrientation_W();
	BOOST_CHECK_SMALL(q.x() - 0., epsilon);
	BOOST_CHECK_SMALL(q.y() - 0., epsilon);
	BOOST_CHECK_SMALL(q.z() - 1., epsilon);
	BOOST_CHECK_SMALL(q.w() - 0., epsilon);


	// Testing another quaternion in the world frame
	ws.setBaseRPY_W(Eigen::Vector3d(0., M_PI_4, M_PI));
	q = ws.getBaseOrientation_W();
	BOOST_CHECK_SMALL(q.x() + 0.382683, epsilon);
	BOOST_CHECK_SMALL(q.y() - 0., epsilon);
	BOOST_CHECK_SMALL(q.z() - 0.92388, epsilon);
	BOOST_CHECK_SMALL(q.w() - 0., epsilon);


	// Testing another quaternion in the horizontal frame
	q = ws.getBaseOrientation_H();
	BOOST_CHECK_SMALL(q.x() - 0., epsilon);
	BOOST_CHECK_SMALL(q.y() - 0., epsilon);
	BOOST_CHECK_SMALL(q.z() - 1., epsilon);
	BOOST_CHECK_SMALL(q.w() - 0., epsilon);


	// Testing the base angular velocity
	ws.setBaseRPY_W(Eigen::Vector3d(0., 0., 0.));
	old_base_state = Eigen::Vector3d(0.5, 0.7, 0.);
	ws.setBaseRotationRate_W(old_base_state);
	new_base_state = ws.getBaseRotationRate_W();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseRotationRate_B();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseRotationRate_H();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);


	// Testing the base acceleration
	old_base_state = Eigen::Vector3d(0.8, 0.9, -9.8);
	ws.setBaseRotAcceleration_W(old_base_state);
	new_base_state = ws.getBaseRotAcceleration_W();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseRotAcceleration_B();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);

	new_base_state = ws.getBaseRotAcceleration_H();
	for (unsigned int i = 0; i < 2; i++)
		BOOST_CHECK_SMALL((double) (new_base_state(i) - old_base_state(i)), epsilon);
}


BOOST_AUTO_TEST_CASE(base_joint) // specify a test case for joint states
{
	dwl::WholeBodyState ws(12);
	unsigned int n_joints;
	unsigned int idx;
	double joint_value;
	Eigen::Vector2d old_joint_state;
	Eigen::Vector2d new_joint_state;


	n_joints = ws.getJointDof();
	BOOST_CHECK(n_joints == 12);

	ws.setJointDoF(2);
	n_joints = ws.getJointDof();
	BOOST_CHECK(n_joints == 2);

	// Testing joint position
	idx = 1;
	old_joint_state = Eigen::Vector2d(0.1, 0.5);
	ws.setJointPosition(old_joint_state);
	joint_value = ws.getJointPosition(idx);
	new_joint_state = ws.getJointPosition();
	BOOST_CHECK_SMALL(joint_value - (double) old_joint_state(idx), epsilon);
	for (unsigned int i = 0; i < old_joint_state.size(); i++)
		BOOST_CHECK_SMALL((double) (new_joint_state(i) - old_joint_state(i)), epsilon);


	// Testing joint velocity
	old_joint_state = Eigen::Vector2d(0.7, 0.8);
	ws.setJointVelocity(old_joint_state);
	joint_value = ws.getJointVelocity(idx);
	new_joint_state = ws.getJointVelocity();
	BOOST_CHECK_SMALL(joint_value - (double) old_joint_state(idx), epsilon);
	for (unsigned int i = 0; i < old_joint_state.size(); i++)
		BOOST_CHECK_SMALL((double) (new_joint_state(i) - old_joint_state(i)), epsilon);


	// Testing joint acceleration
	old_joint_state = Eigen::Vector2d(1.7, 2.8);
	ws.setJointAcceleration(old_joint_state);
	joint_value = ws.getJointAcceleration(idx);
	new_joint_state = ws.getJointAcceleration();
	BOOST_CHECK_SMALL(joint_value - (double) old_joint_state(idx), epsilon);
	for (unsigned int i = 0; i < old_joint_state.size(); i++)
		BOOST_CHECK_SMALL((double) (new_joint_state(i) - old_joint_state(i)), epsilon);


	// Testing joint effort
	old_joint_state = Eigen::Vector2d(17, 28);
	ws.setJointEffort(old_joint_state);
	joint_value = ws.getJointEffort(idx);
	new_joint_state = ws.getJointEffort();
	BOOST_CHECK_SMALL(joint_value - (double) old_joint_state(idx), epsilon);
	for (unsigned int i = 0; i < old_joint_state.size(); i++)
		BOOST_CHECK_SMALL((double) (new_joint_state(i) - old_joint_state(i)), epsilon);
}
