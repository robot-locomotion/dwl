#ifndef DWL__LOCOMOTION__PLANNING_OF_MOTION_SEQUENCE__H
#define DWL__LOCOMOTION__PLANNING_OF_MOTION_SEQUENCE__H

#include <dwl/robot/Robot.h>
#include <dwl/locomotion/MotionPlanning.h>
#include <dwl/locomotion/ContactPlanning.h>
#include <dwl/solver/SearchTreeSolver.h>
#include <dwl/environment/TerrainMap.h>

#include <dwl/utils/utils.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class PlanningOfMotionSequence
 * @brief Abstract class for solving the planning of motion sequence problem.
 * This abstract class allow us to implement different approaches such as:
 * decoupled and coupled approaches. For instance, coupled approaches required
 * one Solver, in contrast of decoupled approaches that require the
 * MotionPlanning and ContactPlanning classes.
 */
class PlanningOfMotionSequence //TODO clean and define the final class
{
	public:
		/** @brief Constructor function */
		PlanningOfMotionSequence();

		/** @brief Destructor function */
		virtual ~PlanningOfMotionSequence();

		/**
		 * @brief Defines the settings required for a decoupled approach
		 * @param Robot* The robot defines all the properties of the robot
		 * @param SearchTreeSolver* The search tree solver that computes a motion plan
		 * @param TerrainMap* Encapsulates all the terrain information
		 */
		void reset(robot::Robot* robot,
				   solver::SearchTreeSolver* solver,
				   environment::TerrainMap* environment);

		/**
		 * @brief Defines the settings required for a decoupled approach
		 * @param Robot* The robot defines all the properties of the robot
		 * @param BodyPlanner* A body planner could computed body paths, poses or/and trajectories
		 * @param ContactPlanning* A contact planner computes the contact sequence
		 * @param TerrainMap* Encapsulates all the terrain information
		 */
		void reset(robot::Robot* robot,
				   MotionPlanning* motion_planner,
				   ContactPlanning* contact_planner,
				   environment::TerrainMap* environment);

		/**
		 * @brief Initializes the planner
		 * @param True if it was initialized the planner
		 */
		bool initPlan();

		/**
		 * @brief Abstract method for initialization of a plan
		 * @return True if it was initialized the planner
		 */
		virtual bool init() = 0;

		/**
		 * @brief Updates the start and goal pose of the robot
		 * @param Pose Goal pose
		 */
		void resetGoal(Pose goal);

		/**
		 * @brief Computes the motion plan
		 * @param Pose Robot pose
		 * @return True if it was computed the plan
		 */
		bool computePlan(Pose robot_state);

		/**
		 * @brief Abstract method for the computation a motion plan according to added constraints
		 * and costs in the optimization problem
		 * @param Pose Robot pose
		 * @return True if it was found a plan
		 */
		virtual bool compute(Pose robot_state) = 0;

		/**
		 * @brief Sets the terrain map
		 * @param TerrainData Terrain map of the environment
		 */
		void setTerrainMap(const TerrainData& terrain_map);

		/**
		 * @brief Sets the obstacle terrain map
		 * @param std::vector<Cell> Obstacle map of the environment
		 */
		void setObstacleMap(std::vector<Cell> obstacle_map);

		/**
		 * @brief Sets the allowed computation time for a coupled planner
		 * @param double Allowed computation time
		 */
		void setComputationTime(double computation_time);

		/**
		 * @brief Sets the allowed computation time for a decoupled planner
		 * @param double Allowed computation time
		 * @param TypeOfSolver Type of solver to set the allowed computation time
		 */
		void setComputationTime(double computation_time, TypeOfSolver solver);

		/**
		 * @brief Gets the approximated body path
		 * @return The approximated body path
		 */
		std::vector<Pose> getBodyPath();

		/**
		 * @brief Gets the planned contact sequence
		 * @return The sequence of contacts
		 */
		std::vector<Contact> getContactSequence();

		/**
		 * @brief Gets the name of the planner
		 * @return The name of the planner
		 */
		std::string getName();


	protected:
		/** @brief Name of the planner */
		std::string name_;

		/** @brief Pointer to the motion planner */
		MotionPlanning* motion_planner_;

		/** @brief Pointer to the contact planner */
		ContactPlanning* contact_planner_;

		/** @brief Pointer to the robot properties */
		robot::Robot* robot_;

		/** @brief Pointer to the solver algorithm */
		solver::SearchTreeSolver* solver_;

		/** @brief Pointer to the terrain information */
		environment::TerrainMap* terrain_;

		/** @brief Initial pose of the robot */
		Pose initial_pose_;

		/** @brief Goal pose of the robot */
		Pose goal_pose_;

		/** @brief Vector of contact points */
		std::vector<Contact> contacts_sequence_;

		/** @brief Vector of body position */
		std::vector<Pose> body_path_;

		/** @brief Computation time */
		double computation_time_;


	private:
		/** @brief Indicates if it was set a solver algorithm for the computation of a plan */
		bool is_set_solver_;

		/** @brief Indicates it was initialized the planning algorithm */
		bool is_initialized_planning_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
