#ifndef DWL__URDF_MODEL__H
#define DWL__URDF_MODEL__H

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Dense>
#include <map>
#include <stack>


namespace dwl
{

namespace urdf_model
{

typedef std::map<std::string,unsigned int> JointID;
typedef std::map<std::string,Eigen::Vector3d> JointAxis;
typedef std::map<std::string,unsigned int> LinkID;
enum JointType {free=0, fixed, floating, all};
enum JointMotion {AX = 0, AY, AZ, LX, LY, LZ, FULL};

/**
 * @brief Gets the joint names from URDF model. By default free joints are get but it's possible
 * to get free, fixed, floating or all joints
 * @param JointID& Joint ids and names
 * @param std::string URDF model
 */
void getJointNames(JointID& joints,
				   std::string urdf_model,
				   enum JointType type = free);

/**
 * @brief Get the end-effector names from URDF model
 * @param LinkID& End-effector link ids and names
 * @param std::string URDF model
 */
void getEndEffectors(LinkID& end_effectors,
					 std::string urdf_model);

/**
 * @brief Get the joint limits from URDF model. Floating-base joint are not considered as joints
 * @param Eigen::VectorXd& Lower joint position limits
 * @param Eigen::VectorXd& Upper joint position limits
 * @param Eigen::VectorXd& Joint velocity limits
 * @param Eigen::VectorXd& Joint effort limits
 */
void getJointLimits(Eigen::VectorXd& lower_joint_pos,
					Eigen::VectorXd& upper_joint_pos,
					Eigen::VectorXd& joint_vel,
					Eigen::VectorXd& joint_eff,
					std::string urdf_model);

/**
 * @brief Gets the joint axis from URDF model. By default free joints are get but it's possible
 * to get free, fixed, floating or all joints
 * @param JointAxis& Joint axis and names
 * @param std::string URDF model
 */
void getJointAxis(JointAxis& joints,
				  std::string urdf_model,
				  enum JointType type = free);

/**
 * @brief Gets the joint type of motion from URDF model. By default free joints are get but it's
 * possible to get free, fixed, floating or all joints
 * @param JointID& Joint type of motion
 * @param std::string URDF model
 */
void getFloatingBaseJointMotion(JointID& joints,
								std::string urdf_model);

} //@namespace urdf_model
} //@namespace dwl

#endif
