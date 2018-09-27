#include <dwl/model/WholeBodyKinematics.h>
#define BOOST_TEST_MODULE DWL_TESTS
#include <boost/test/included/unit_test.hpp>



BOOST_AUTO_TEST_CASE(test_hyq) // specify a test case for hyq properties
{
    std::string urdf_file = DWL_SOURCE_DIR"/models/hyq.urdf";
    std::string yarf_file = DWL_SOURCE_DIR"/models/hyq.yarf";
    dwl::model::FloatingBaseSystem fbs;
    dwl::model::WholeBodyKinematics wkin;
    fbs.resetFromURDFFile(urdf_file, yarf_file);
    wkin.reset(fbs);


    // Creating an initial state for testing
    dwl::SE3 base_pos;
    dwl::Motion base_vel, base_acc;
    base_vel.setLinear(Eigen::Vector3d(1., 0., 0.));
    base_acc.setAngular(Eigen::Vector3d(0., 1., 0.));
    Eigen::VectorXd joint_pos = fbs.getDefaultPosture();
    Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(fbs.getJointDoF());
    Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(fbs.getJointDoF());


    // Checking the CoM position and velocity
    Eigen::Vector3d x, xd, xdd;
    wkin.computeCoMRate(x, xd, xdd,
						base_pos, joint_pos,
						base_vel, joint_vel,
						base_acc, joint_acc);
    BOOST_CHECK( x.isApprox(Eigen::Vector3d(-0.000278458, 0., -0.0460617), 1e-3) );
    BOOST_CHECK( xd.isApprox(Eigen::Vector3d(1., 0., 0.)) );


    // Checking the forward kinematics for the LF foot
    dwl::SE3Map pos_W =
        wkin.computePosition(base_pos, joint_pos,
                             fbs.getEndEffectorList());
    dwl::SE3 lf_pos(Eigen::Vector3d(0.370773, 0.324067, -0.57751),
    				Eigen::Vector4d(0.0928958, -0.364443, -0.0365662, 0.925859));
    BOOST_CHECK( pos_W.find("lf_foot")->second.getTranslation().isApprox(lf_pos.getTranslation(), 1e-3) );
    BOOST_CHECK( pos_W.find("lf_foot")->second.getQuaternion().isApprox(lf_pos.getQuaternion(), 1e-3) );


    // Checking the LF foot velocity
    dwl::MotionMap vel_W =
        wkin.computeVelocity(base_pos, joint_pos,
                             base_vel, joint_vel,
                             fbs.getEndEffectorList());
    dwl::Motion lf_vel(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.));
    BOOST_CHECK( vel_W.find("lf_foot")->second.getLinear().isApprox(lf_vel.getLinear(), 1e-12) );
    BOOST_CHECK( vel_W.find("lf_foot")->second.getAngular().isApprox(lf_vel.getAngular(), 1e-12) );


    // Checking the LF foot acceleration
    dwl::MotionMap acc_W =
        wkin.computeAcceleration(base_pos, joint_pos,
                                 base_vel, joint_vel,
                                 base_acc, joint_acc,
                                 fbs.getEndEffectorList());
    dwl::Motion lf_acc(Eigen::Vector3d(-0.94828, 0., 0.206736), Eigen::Vector3d(0., 1., 0.));
    BOOST_CHECK( acc_W.find("lf_foot")->second.getLinear().isApprox(lf_acc.getLinear(), 1e-3) );
    BOOST_CHECK( acc_W.find("lf_foot")->second.getAngular().isApprox(lf_acc.getAngular(), 1e-3) );


    // Checking the fixed-based IK
    Eigen::VectorXd new_joint_pos = Eigen::VectorXd::Zero(fbs.getJointDoF());
    dwl::SE3Map pos_B = pos_W; // Note that we didn't move the base configuration
    wkin.computeJointPosition(new_joint_pos, pos_B);
    BOOST_CHECK( new_joint_pos.isApprox(joint_pos, 1e-12) );


    // Checking the joint velocity computation
    Eigen::VectorXd new_joint_vel = Eigen::VectorXd::Zero(fbs.getJointDoF());
    joint_vel.setRandom();
    dwl::MotionMap vel_B =
        wkin.computeVelocity(base_pos, joint_pos,
                             dwl::Motion(), joint_vel,
                             fbs.getEndEffectorList());
	wkin.computeJointVelocity(new_joint_vel, joint_pos, vel_B);
    BOOST_CHECK( new_joint_vel.isApprox(joint_vel, 1e-12) );


    // Checking the joint velocity computation
    Eigen::VectorXd new_joint_acc = Eigen::VectorXd::Zero(fbs.getJointDoF());
    joint_acc.setRandom();
    dwl::MotionMap acc_B =
        wkin.computeAcceleration(base_pos, joint_pos,
                                 dwl::Motion(), joint_vel,
                                 dwl::Motion(), joint_acc,
                                 fbs.getEndEffectorList());
	wkin.computeJointAcceleration(new_joint_acc, joint_pos, joint_vel, acc_B);
    BOOST_CHECK( new_joint_acc.isApprox(joint_acc, 1e-12) );
std::cout << joint_acc.transpose() << std::endl;
std::cout << new_joint_acc.transpose() << std::endl;
}
