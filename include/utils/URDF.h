#ifndef DWL__URDF_MODEL__H
#define DWL__URDF_MODEL__H

#include <utils/RigidBodyDynamics.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <map>
#include <stack>


namespace dwl
{

namespace urdf_model
{

typedef rbd::BodyID JointID;

/**
 * @brief Get the joint names from URDF model. Floating-base joint are not considered as joints
 * @param JointID& Joint ids and names
 * @param std::string URDF model
 * @param struct FloatingBaseSystem* Defines the general properties of a floating-base
 * system
 */
void getJointNames(JointID& joints,
				   std::string urdf_model,
				   rbd::FloatingBaseSystem* system);

/**
 * @brief Get the end-effector names from URDF model
 * @param rbd::BodySelector& End-effector names
 * @param std::string URDF model
 * @param struct FloatingBaseSystem* Defines the general properties of a floating-base
 * system
 */
void getEndEffectors(rbd::BodySelector& end_effectors,
					 std::string urdf_model,
					 rbd::FloatingBaseSystem* system);

/**
 * @brief Get the joint limits from URDF model. Floating-base joint are not considered as joints
 * @param Eigen::VectorXd& Lower joint position limits
 * @param Eigen::VectorXd& Upper joint position limits
 * @param Eigen::VectorXd& Joint velocity limits
 * @param Eigen::VectorXd& Joint effort limits
 * @param struct FloatingBaseSystem* Defines the general properties of a floating-base
 * system
 */
void getJointLimits(Eigen::VectorXd& lower_joint_pos,
					Eigen::VectorXd& upper_joint_pos,
					Eigen::VectorXd& joint_vel,
					Eigen::VectorXd& joint_eff,
					std::string urdf_model,
					rbd::FloatingBaseSystem* system);

} //@namespace urdf_model
} //@namespace dwl

#endif
