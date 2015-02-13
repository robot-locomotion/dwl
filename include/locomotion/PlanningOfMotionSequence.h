#ifndef DWL_PlanningOfMotionSequence_H
#define DWL_PlanningOfMotionSequence_H

#include <robot/Robot.h>
#include <locomotion/MotionPlanning.h>
#include <locomotion/ContactPlanning.h>
#include <solver/Solver.h>
#include <locomotion/Constraint.h>
#include <locomotion/Cost.h>
#include <environment/EnvironmentInformation.h>

#include <utils/utils.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class PlanningOfMotionSequence
 * @brief Abstract class for solving the planning of motion sequence problem. This abstract class allow us to
 * implement different approaches such as: decoupled and coupled approaches. For instance, coupled approaches
 * required a one Solver, in contrast of decoupled approaches that require the MotionPlanning and
 * ContactPlanning classes.
 */
class PlanningOfMotionSequence
{
	public:
		/** @brief Constructor function */
		PlanningOfMotionSequence();

		/** @brief Destructor function */
		virtual ~PlanningOfMotionSequence();

		/**
		 * @brief Defines the settings required for a decoupled approach
		 * @param Robot* The robot defines all the properties of the robot
		 * @param Solver* The solver that computes a motion plan, e.g. graph-searcher or optimizer
		 * @param EnvironmentInformation* Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, solver::Solver* solver, environment::EnvironmentInformation* environment);

		/**
		 * @brief Defines the settings required for a decoupled approach
		 * @param Robot* The robot defines all the properties of the robot
		 * @param BodyPlanner* A body planner could computed body paths, poses or/and trajectories
		 * @param ContactPlanning* A contact planner computes the contact sequence
		 * @param EnvironmentInformation* Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, MotionPlanning* motion_planner, ContactPlanning* contact_planner,
					environment::EnvironmentInformation* environment);

		/**
		 * @brief Adds an active or inactive constraints to the planning algorithm
		 * @param Constraint* Constraint to add it
		 */
		void addConstraint(Constraint* constraint);

		/**
		 * @brief Removes an active or inactive constraints to the planning algorithm
		 * @param Constraint* Constraint to remove it
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost function for the planning algorithm
		 * @param Cost* Cost to add it
		 */
		void addCost(Cost* cost);

		/**
		 * @brief Removes a cost function for the planning algorithm
		 * @param Cost* Cost to remove it
		 */
		void removeCost(std::string cost_name);

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
		virtual void resetGoal(Pose goal) = 0;

		/**
		 * @brief Computes the motion plan
		 * @param Pose Robot pose
		 * @return True if it was computed the plan
		 */
		bool computePlan(Pose robot_state);

		/**
		 * @brief Abstract method for the computation a motion plan according to added constraints and costs
		 * in the optimization problem
		 * @param Pose Robot pose
		 * @return True if it was found a plan
		 */
		virtual bool compute(Pose robot_state) = 0;

		/**
		 * @brief Sets the reward information of the environment inside of the pointer of the
		 * EnvironmentInformation object, which could be used for different solvers
		 * @param std::vector<Cell> Reward map of the environment
		 */
		void setEnvironmentInformation(std::vector<RewardCell> reward_map);

		/**
		 * @brief Sets the obstacle information of the environment inside of the pointer of the
		 * EnvironmentInformation object, which could be used for different solvers
		 * @param std::vector<Cell> Obstacle map of the environment
		 */
		void setEnvironmentInformation(std::vector<Cell> obstacle_map);

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
		solver::Solver* solver_;

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
		/** @brief Indicates if it was set a solver algorithm for the computation of a plan */
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

} //@namespace locomotion
} //@namespace dwl

#endif
