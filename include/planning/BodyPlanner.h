#ifndef DWL_BodyPlanner_H
#define DWL_BodyPlanner_H

#include <planning/Solver.h>
#include <environment/EnvironmentInformation.h>
#include <utils/utils.h>
#include <utils/Orientation.h>


namespace dwl
{

namespace planning
{

/**
 * @class BodyPlanner
 * @brief Class for implementing a body planner
 */
class BodyPlanner
{
	public:
		/** @brief Constructor function */
		BodyPlanner();

		/** @brief Destructor function */
		~BodyPlanner();

		/**
		 * @brief Specifies the environment information
		 * @param dwl::robot::Robot* robot Encapsulates all the properties of the robot
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, environment::EnvironmentInformation* environment);

		/**
		 * @brief Specifies the body path planner
		 * @param dwl::planning::Solver* path_planner Body path planner
		 */
		void reset(Solver* path_planner);

		/**
		 * @brief Computes the body path from goal pose
		 * @param std::vector<dwl::Pose>& body_path Body path
		 * @param dwl::Pose start_pose Start pose
		 * @param dwl::Pose goal_pose Goal pose
		 */
		bool computeBodyPath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose);

		/**
		 * @brief Sets the computation time
		 * @param double computation_time Computation time
		 * @param bool path_solver True indicates that is a path solver, false that is a pose solver
		 */
		void setComputationTime(double computation_time, bool path_solver);


	protected:
		/** @brief Pointer to the EnvironmentInformation object */
		environment::EnvironmentInformation* environment_;

		/** @brief Pointer to the Robot object */
		robot::Robot* robot_;

		/** @brief Pointer to the path solver */
		Solver* path_solver_;

		/** @brief Pointer to the pose solver */
		Solver* pose_solver_;

		/** @brief Computation time for the path solver */
		double path_computation_time_;

		/** @brief Computation time for the pose solver */
		double pose_computation_time_;
};

} //@namespace planning
} //@namespace dwl

#endif
