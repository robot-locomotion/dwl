#ifndef DWL_MotionPlanning_H
#define DWL_MotionPlanning_H

#include <solver/Solver.h>
#include <environment/EnvironmentInformation.h>
#include <utils/utils.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class MotionPlanning
 * @brief Abstract class for implementing a motion planning. This abstract class has two methods reset() that
 * allow us to define Robot properties, EnvironmentInformation, and the Solver of the planning
 */
class MotionPlanning
{
	public:
		/** @brief Constructor function */
		MotionPlanning();

		/** @brief Destructor function */
		virtual ~MotionPlanning();

		/**
		 * @brief Defines the robot and environment information
		 * @param Robot* Encapsulates all the properties of the robot
		 * @param EnvironmentInformation* Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, environment::EnvironmentInformation* environment);

		/**
		 * @brief Defines the motion planning solver
		 * @param Solver* Motion planning solver
		 */
		void reset(solver::Solver* solver);

		/**
		 * @brief Computes a path from start pose to goal pose
		 * @param std::vector<Pose>& Planned path
		 * @param Pose Start pose
		 * @param Pose Goal pose
		 */
		virtual bool computePath(std::vector<Pose>& path, Pose start_pose, Pose goal_pose) = 0;

		/**
		 * @brief Sets the computation time
		 * @param double Computation time
		 * @param bool True indicates that is a path solver, false that is a pose solver
		 */
		void setComputationTime(double computation_time, bool path_solver);


	protected:
		/** @brief Name of the motion planner */
		std::string name_;

		/** @brief Pointer to the EnvironmentInformation object */
		environment::EnvironmentInformation* environment_;

		/** @brief Pointer to the Robot object */
		robot::Robot* robot_;

		/** @brief Pointer to the path solver */
		solver::Solver* path_solver_;

		/** @brief Pointer to the pose solver */
		solver::Solver* pose_solver_;

		/** @brief Computation time for the path solver */
		double path_computation_time_;

		/** @brief Computation time for the pose solver */
		double pose_computation_time_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
