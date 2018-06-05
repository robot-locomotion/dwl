#include <dwl/model/WholeBodyKinematics.h>
#define BOOST_TEST_MODULE DWL_TESTS
#include <boost/test/included/unit_test.hpp>


// Tolerance
double epsilon = 1e-3;

BOOST_AUTO_TEST_CASE(test_hyq) // specify a test case for hyq properties
{
    std::string urdf_file = DWL_SOURCE_DIR"/models/hyq.urdf";
    std::string yarf_file = DWL_SOURCE_DIR"/models/hyq.yarf";
    dwl::model::FloatingBaseSystem fbs;
    dwl::model::WholeBodyKinematics wkin;
    fbs.resetFromURDFFile(urdf_file, yarf_file);
    wkin.reset(fbs);


    // Creating an initial state for testing
    Eigen::Vector7d base_pos;
    Eigen::Vector6d base_vel, base_acc;
    base_pos << 0., 0., 0., 1., 0., 0., 0.;
    base_vel << 0., 0., 0., 1., 0., 0.;
    base_acc << 0., 1., 0., 0., 0., 0.;
    Eigen::VectorXd joint_pos = fbs.getDefaultPosture();
    Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(fbs.getJointDoF());
    Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(fbs.getJointDoF());


    // Checking the CoM position and velocity
    Eigen::Vector3d x, xd;
    wkin.computeCoMRate(x, xd,
						base_pos, joint_pos,
						base_vel, joint_vel);
    BOOST_CHECK( x.isApprox(Eigen::Vector3d(-0.000278458, 0., -0.0460617), epsilon) );
    BOOST_CHECK( xd.isApprox(Eigen::Vector3d(1., 0., 0.)) );


    // Checking the forward kinematics for the LF foot
    Eigen::Vector7dMap pos_W =
        wkin.computePosition(base_pos, joint_pos,
                             fbs.getEndEffectorNames());
    Eigen::Vector7d lf_pos;
    lf_pos << 0.0928958, -0.364443, -0.0365662, 0.925859, 0.370773, 0.324067, -0.57751;
    BOOST_CHECK( pos_W.find("lf_foot")->second.isApprox(lf_pos, epsilon) );


    // Checking the LF foot velocity
    Eigen::Vector6dMap vel_W = 
        wkin.computeVelocity(base_pos, joint_pos,
                             base_vel, joint_vel,
                             fbs.getEndEffectorNames());
    Eigen::Vector6d lf_vel;
    lf_vel << 0., 0., 0., 1., 0., 0.;
    BOOST_CHECK( vel_W.find("lf_foot")->second.isApprox(lf_vel, epsilon) );


    // Checking the LF foot acceleration
    Eigen::Vector6dMap acc_W = 
        wkin.computeAcceleration(base_pos, joint_pos,
                                 base_vel, 0.*joint_vel,
                                 base_acc, joint_acc,
                                 fbs.getEndEffectorNames());
    Eigen::Vector6d lf_acc;
    lf_acc << 0., 1., 0., -0.57751, 0., -0.370773;
    BOOST_CHECK( acc_W.find("lf_foot")->second.isApprox(lf_acc, epsilon) );


    // Checking the fixed-based IK
    Eigen::VectorXd new_joint_pos;
    Eigen::Vector7dMap pos_B = pos_W; // Note that we didn't move the base configuration
    wkin.computeJointPosition(new_joint_pos, pos_B);
    BOOST_CHECK( new_joint_pos.isApprox(joint_pos) );


    // Checking the joint velocity computation
    Eigen::VectorXd new_joint_vel;
    joint_vel.setRandom();
    Eigen::Vector6dMap vel_B = 
        wkin.computeVelocity(base_pos, joint_pos,
                             Eigen::Vector6d::Zero(), joint_vel,
                             fbs.getEndEffectorNames());
	wkin.computeJointVelocity(new_joint_vel, joint_pos, vel_B);
    BOOST_CHECK( new_joint_vel.isApprox(joint_vel) );


    // Checking the joint velocity computation
    Eigen::VectorXd new_joint_acc;
    joint_acc.setRandom();
    Eigen::Vector6dMap acc_B = 
        wkin.computeAcceleration(base_pos, joint_pos,
                                 Eigen::Vector6d::Zero(), joint_vel,
                                 Eigen::Vector6d::Zero(), joint_acc,
                                 fbs.getEndEffectorNames());
	wkin.computeJointAcceleration(new_joint_acc, joint_pos, joint_vel, acc_B);
    BOOST_CHECK( new_joint_acc.isApprox(joint_acc) );
//     std::cout << joint_acc.transpose() << std::endl;
//     std::cout << new_joint_acc.transpose() << std::endl;

// std::cout << std::endl;
// std::cout << acc_B.find("lf_foot")->second.transpose() << std::endl;
// std::cout << wkin.getFrameJdQd("lf_foot").transpose() << std::endl;

}