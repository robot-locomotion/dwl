#include <dwl/model/FloatingBaseSystem.h>
#define BOOST_TEST_MODULE DWL_TESTS
#include <boost/test/included/unit_test.hpp>

// Tolerance
double epsilon = 0.00001;


BOOST_AUTO_TEST_CASE(test_hyq) // specify a test case for hyq properties
{
    // Floating-base system
    std::string model_file = DWL_SOURCE_DIR"/models/hyq.urdf";
    std::string robot_file = DWL_SOURCE_DIR"/models/hyq.yarf";
    dwl::model::FloatingBaseSystem fbs;
    fbs.resetFromURDFFile(model_file, robot_file);


    // Checking the dimensions of the robot's manifolds and states
    BOOST_CHECK( fbs.getConfigurationDim() == 19 );
    BOOST_CHECK( fbs.getTangentDim() == 18 );
    BOOST_CHECK( fbs.getJointDoF() == 12 );
    
    
    // Checking the name of the floating-base
    BOOST_CHECK( fbs.getFloatingBaseName() == "trunk" );
    

    // Checking joint indexes and properties
    for (unsigned int j = 0; j < fbs.getJointDoF(); ++j) {
        std::string name = fbs.getJointName(j);
        BOOST_CHECK( fbs.getJointId(name) == j );
        BOOST_CHECK( fbs.getJointNames()[j] == name );
        BOOST_CHECK( fbs.existJoint(name) == true );
        BOOST_CHECK( fbs.existJoint(name + "n$#1!") == false );
    }


    // Checking the default posture
    Eigen::VectorXd joint_posture = Eigen::VectorXd::Zero(fbs.getJointDoF());
    joint_posture << -0.2, 0.75, -1.5, -0.2, -0.75, 1.5, -0.2, 0.75, -1.5, -0.2, -0.75, 1.5;
    BOOST_CHECK( joint_posture.isApprox(fbs.getDefaultPosture()) );


    // Checking body indexes
    dwl::model::ElementId bodies = fbs.getBodies();
    BOOST_CHECK( fbs.getBodyId("trunk") == 1 );
    BOOST_CHECK( fbs.getBodyId("base_link") == 1 );
    BOOST_CHECK( fbs.getBodyId("lf_hipassembly") == 2 );
    BOOST_CHECK( fbs.getBodyId("lf_upperleg") == 3 );
    BOOST_CHECK( fbs.getBodyId("lf_lowerleg") == 4 );
    BOOST_CHECK( fbs.getBodyId("lf_foot") == 4 );
    BOOST_CHECK( fbs.getBodyId("lh_foot") == 7 );
    BOOST_CHECK( fbs.getBodyId("rf_foot") == 10 );
    BOOST_CHECK( fbs.getBodyId("rh_foot") == 13 );


    // Checking various masses of the robot
    BOOST_CHECK_SMALL( fbs.getTotalMass() - 84.896,  epsilon );
    BOOST_CHECK_SMALL( fbs.getFloatingBaseMass() - 59.1,  epsilon );
    BOOST_CHECK_SMALL( fbs.getBodyMass("trunk") - fbs.getFloatingBaseMass(),  epsilon );
    BOOST_CHECK_SMALL( fbs.getBodyMass("lf_hipassembly") - 2.93,  epsilon );
    BOOST_CHECK_SMALL( fbs.getBodyMass("lf_upperleg") - 2.638,  epsilon );
    BOOST_CHECK_SMALL( fbs.getBodyMass("lf_lowerleg") - 0.881,  epsilon );


    // Checking the robot's CoM values
    BOOST_CHECK( fbs.getFloatingBaseCoM().isApprox(Eigen::Vector3d(-0.0004, -5.36e-05, 0.003553)) );
    BOOST_CHECK( fbs.getBodyCoM("lf_hipassembly").isApprox(Eigen::Vector3d(0.04263, 0., 0.16931)) );
    BOOST_CHECK( fbs.getBodyCoM("lf_upperleg").isApprox(Eigen::Vector3d(0.15074, -0.02625, 0.)) );
    BOOST_CHECK( fbs.getBodyCoM("lf_lowerleg").isApprox(Eigen::Vector3d(0.1254, 4e-05, -0.0001)) );


    // Checking the robot's end-effector
    BOOST_CHECK( fbs.getNumberOfEndEffectors() == 4);
    BOOST_CHECK( fbs.getEndEffectorId("lf_foot") == 0 );
    BOOST_CHECK( fbs.getEndEffectorId("lh_foot") == 1 );
    BOOST_CHECK( fbs.getEndEffectorId("rf_foot") == 2 );
    BOOST_CHECK( fbs.getEndEffectorId("rh_foot") == 3 );


    // Checking the gravity vector
    BOOST_CHECK( fbs.getGravityVector().isApprox(Eigen::Vector3d(0., 0., -9.81)) );
    BOOST_CHECK_SMALL( fbs.getGravityAcceleration() - 9.81, epsilon );

 
    // Checking the type of system
    BOOST_CHECK( fbs.isFloatingBase() == true );
    BOOST_CHECK( fbs.isFixedBase() == false );

    
    // Checking conversion between configuration states
    Eigen::Vector7d base_se3, base_pos;
    base_se3 << 0., 0., 0., 1., 0.2, 0.3, -0.6; // base_se3 = [base_quat, base_pos]
    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(fbs.getJointDoF());
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(fbs.getConfigurationDim());
    q0 << 0.2, 0.3, -0.6, 0., 0., 0., 1., joint_posture; // q = [base_pos, base_quat, joint_pos]
    BOOST_CHECK( fbs.toConfigurationState(base_se3, joint_posture).isApprox(q0) );
    fbs.fromConfigurationState(base_pos, joint_pos, q0);
    BOOST_CHECK( base_pos.isApprox(base_se3) );
    BOOST_CHECK( joint_pos.isApprox(joint_posture) );
    
    
    // Checking conversion between tangent states
    Eigen::Vector6d base_se3_vel, base_vel;
    base_se3_vel << 0.8, 0.3, -0.6, 0.2, 0.3, -0.6; // base_se3_vel = [base_ang_vel, base_lin_vel]
    Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(fbs.getJointDoF());
    Eigen::VectorXd qd0 = Eigen::VectorXd::Zero(fbs.getTangentDim());
    qd0 << 0.2, 0.3, -0.6, 0.8, 0.3, -0.6, 0.01 * joint_posture; // qd = [base_lin_vel, base_ang_vel, joint_vel]
    BOOST_CHECK( fbs.toTangentState(base_se3_vel, 0.01 * joint_posture).isApprox(qd0) );
    fbs.fromTangentState(base_vel, joint_vel, qd0);
    BOOST_CHECK( base_vel.isApprox(base_se3_vel) );
    BOOST_CHECK( joint_vel.isApprox(0.01 * joint_posture) );


    // Checking the position index and dimension of all kinematic branches
    unsigned int pos_idx, n_dof;
    fbs.getBranch(pos_idx, n_dof, "lf_foot");
    BOOST_CHECK( pos_idx == 0 );
    BOOST_CHECK( n_dof == 3 );
    fbs.getBranch(pos_idx, n_dof, "lh_foot");
    BOOST_CHECK( pos_idx == 3 );
    BOOST_CHECK( n_dof == 3 );
    fbs.getBranch(pos_idx, n_dof, "rf_foot");
    BOOST_CHECK( pos_idx == 6 );
    BOOST_CHECK( n_dof == 3 );
    fbs.getBranch(pos_idx, n_dof, "rh_foot");
    BOOST_CHECK( pos_idx == 9 );
    BOOST_CHECK( n_dof == 3 );

}