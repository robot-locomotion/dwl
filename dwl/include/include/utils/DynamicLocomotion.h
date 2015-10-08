#ifndef DWL__DYNAMIC_LOCOMOTION__H
#define DWL__DYNAMIC_LOCOMOTION__H

#include <map>
#include <vector>

#include <Eigen/Dense>
#include <utils/GraphSearching.h>
#include <utils/RigidBodyDynamics.h>


namespace dwl
{

/** @brief Defines a pattern of locomotion mapping */
typedef std::map<unsigned int, unsigned int> PatternOfLocomotionMap;

/** @brief Defines a end-effector map between the id and the name */
typedef std::map<unsigned int, std::string> EndEffectorMap;

/** @brief Defines the patch end-effector elements */
typedef std::map<unsigned int, std::vector<std::string> > PatchMap; //TODO

/** @brief Defines a Vector3d map */
typedef std::map<unsigned int, Eigen::Vector3d> Vector3dMap;

/** @brief Enumarates types of states */
enum TypeOfState {XY, XY_Y, XYZ_RPY};

/** @brief Enumarates types of solvers */
enum TypeOfSolver {BodyPathSolver, BodyPoseSolver, ContactSolver};

/**
 * @brief Struct that defines the body pose
 */
struct Pose
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation; // Quaternion
};

/**
 * @brief Struct that defines the 3d pose, i.e. (x,y) and yaw angle
 */
struct Pose3d
{
	Eigen::Vector2d position;
	double orientation;
};

/**
 * @brief Struct that defines the 3d actions, i.e. 3d pose and cost for action
 */
struct Action3d
{
	Pose3d pose;
	Weight cost;
};

/**
 * @brief Struct that defines the search area
 */
struct SearchArea
{
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double resolution;
};

/** @brief Defines a search area map */
typedef std::map<unsigned int, SearchArea> SearchAreaMap;

/**
 * @struct NeighboringArea
 * @brief Struct that defines the neighboring area
 */
struct NeighboringArea
{
	int min_x, max_x;
	int min_y, max_y;
	int min_z, max_z;
};

/**
 * @struct Contact
 * @brief Struct that defines a contact
 */
struct Contact
{
	int end_effector;
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d force;
};

/**
 * @struct ContactSearchRegion
 * @brief Struct that defines a contact search region
 */
struct ContactSearchRegion
{
	int end_effector;
	Eigen::Vector3d position;
	SearchArea region;
};

/**
 * @struct LocomotionState
 * @brief Defines a locomotion state
 */
struct LocomotionState
{
	LocomotionState(unsigned int num_joints = 0,
					unsigned int num_end_effectors = 0) : time(0.), duration(0.) {
		base_pos.setZero();
		base_vel.setZero();
		base_acc.setZero();
		base_eff.setZero();

		if (num_joints != 0) {
			joint_pos.setZero(num_joints);
			joint_vel.setZero(num_joints);
			joint_acc.setZero(num_joints);
			joint_eff.setZero(num_joints);
		}

		if (num_end_effectors != 0)
			contacts.resize(num_end_effectors);
	}

	void setJointDoF(unsigned int num_joints) {
		joint_pos.setZero(num_joints);
		joint_vel.setZero(num_joints);
		joint_acc.setZero(num_joints);
		joint_eff.setZero(num_joints);
	}

	double time;
	double duration;
	rbd::Vector6d base_pos;
	rbd::Vector6d base_vel;
	rbd::Vector6d base_acc;
	rbd::Vector6d base_eff;
	Eigen::VectorXd joint_pos;
	Eigen::VectorXd joint_vel;
	Eigen::VectorXd joint_acc;
	Eigen::VectorXd joint_eff;
	std::vector<Contact> contacts;
};

/**
 * @struct RobotAndTerrain
 * @brief Struct to define the relevant information of the robot and terrain for computing the reward
 */
struct RobotAndTerrain
{
	Eigen::Vector3d body_action;
	Pose3d pose;
	Contact potential_contact;
	std::vector<Contact> current_contacts;
	std::map<Vertex, double> height_map;
	double resolution;
};

} //@namespace dwl

#endif
