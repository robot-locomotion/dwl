#ifndef DWL_PlanningOfMotionSequences_H
#define DWL_PlanningOfMotionSequences_H

#include <robot/Robot.h>
#include <planning/MotionPlanning.h>
#include <planning/ContactPlanning.h>
#include <planning/Solver.h>
#include <planning/Constraint.h>
#include <planning/Cost.h>
#include <environment/EnvironmentInformation.h>

#include <utils/utils.h>


namespace dwl
{

namespace planning
{

/**
 * @class PlanningOfMotionSequences
 * @brief Abstract class for solving the planning of motion sequence problem (optimization problem)
 */
class PlanningOfMotionSequences
{
	public:
		/** @brief Constructor function */
		PlanningOfMotionSequences();

		/** @brief Destructor function */
		virtual ~PlanningOfMotionSequences();

		/**
		 * @brief Specifies the settings of all components within the decoupled approach for solving Planning of Motion Sequences problem
		 * @param dwl::robot::Robot* robot The robot defines all the properties of the robot
		 * @param dwl::planning::Solver* solver	The solver computes a solution of the motion planning problem which depends of the algorithm, i.e. graph-searching or optimization problems
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, Solver* solver, environment::EnvironmentInformation* environment);

		/**
		 * @brief Specifies the settings of all components within the decoupled approach for solving Planning of Motion Sequences problem
		 * @param dwl::robot::Robot* robot The robot defines all the properties of the robot
		 * @param dwl::planning::BodyPlanner* motion_planner The motion planner computes body path and trajectory, and pose
		 * @param dwl::planning::ContactPlanning* contact_planner The contact planner computes the contact sequence
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, MotionPlanning* motion_planner, ContactPlanning* contact_planner, environment::EnvironmentInformation* environment);

		/**
		 * @brief Adds an active or inactive constraints to the planning algorithm
		 * @param dwl::planning::Constraint* constraint Pointer to the constraint class
		 */
		void addConstraint(Constraint* constraint);

		/**
		 * @brief Removes an active or inactive constraints to the planning algorithm
		 * @param dwl::planning::Constraint* constraint Pointer to the constraint class
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost for the optimization problem of the planning algorithm
		 * @param dwl::planning::Cost* Pointer to the cost class
		 */
		void addCost(Cost* cost);

		/**
		 * @brief Removes a cost for the optimization problem of the planning algorithm
		 * @param dwl::planning::Cost* Pointer to the cost class
		 */
		void removeCost(std::string cost_name);

		/**
		 * @brief Initialized the planner
		 * @param bool Return true if it was initialized the planner
		 */
		bool initPlan();

		/**
		 * @brief Abstract method for initialization of a plan
		 * @param std::vector<double> start Initial state
		 * @param std::vector<double> goal Goal state to arrive
		 */
		virtual bool init() = 0;

		/**
		 * @brief Updates the start and goal pose of the robot
		 * @param dwl::Pose goal Goal pose
		 */
		virtual void resetGoal(Pose goal) = 0;

		/**
		 * @brief Computes the motion planning
		 * @param dwl::planning::Pose robot_state Robot pose
		 * @return bool Return true if it was computed the plan
		 */
		bool computePlan(Pose robot_state);

		/**
		 * @brief Abstract method for the computation a motion plan according to added constraints and costs in the optimization problem
		 * @param dwl::planning::Pose robot_state Robot pose
		 * @return bool Return true if it was found a plan
		 */
		virtual bool compute(Pose robot_state) = 0;

		/**
		 * @brief Sets the reward information of the environment inside of the pointer of the EnvironmentInformation object, which could be used for differents solvers
		 * @param std::vector<Cell> reward_map Reward map of the environment
		 */
		void setEnvironmentInformation(std::vector<RewardCell> reward_map);

		/**
		 * @brief Sets the obstacle information of the environment inside of the pointer of the EnvironmentInformation object, which could be used for differents solvers
		 * @param std::vector<Cell> obstacle_map Obstacle map of the environment
		 */
		void setEnvironmentInformation(std::vector<Cell> obstacle_map);

		/**
		 * @brief Sets the allowed computation time for a coupled planner
		 * @param double computation_time Allowed computation time
		 */
		void setComputationTime(double computation_time);

		/**
		 * @brief Sets the allowed computation time for a decoupled planner
		 * @param double computation_time Allowed computation time
		 * @param dwl::TypeOfSolver solver Type of solver to set the allowed computation time
		 */
		void setComputationTime(double computation_time, TypeOfSolver solver);

		/**
		 * @brief Gets the approximated body path
		 * @return std::vector<Pose> Return the approximated body path
		 */
		std::vector<Pose> getBodyPath();

		std::vector<Contact> getContactSequence();

		/**
		 * @brief Gets the name of the planner
		 * @return std::string Return the name of the planner
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
		Solver* solver_;

		/** @brief Pointer to the environment information */
		environment::EnvironmentInformation* environment_;

		/** @brief Vector of active constraints pointers */
		std::vector<Constraint*> active_constraints_;

		/** @brief Vector of inactive constraints pointers */
		std::vector<Constraint*> inactive_constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

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
		/** @brief Indicates if it was settep a solver algorithm for the computation of a plan */
		bool is_set_solver_;

		/** @brief Indicates it was initialized the planning algorithm */
		bool is_initialized_planning_;

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_active_constraint_;

		/** @brief Indicates if it was added an inactive constraint in the solver */
		bool is_added_inactive_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;
};

} //@namespace planning
} //@namespace dwl

#endif
