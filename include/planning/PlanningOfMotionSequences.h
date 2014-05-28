#ifndef DWL_PlanningOfMotionSequences_H
#define DWL_PlanningOfMotionSequences_H

#include <planning/Solver.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>
#include <pthread.h>


namespace dwl
{

namespace planning
{

/**
 * @brief Struct that defines a contact
 */
struct Contact
{
	int end_effector;
	Eigen::Vector3d position;
};

/**
 * @brief Struct that defines the body pose
 */
struct BodyPose
{
	Eigen::Vector3d position;
	Eigen::Vector3d orientation;
};

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
		virtual ~PlanningOfMotionSequences() {}

		/**
		 * @brief Function to specify the settings of all variables within the Planning of Motion Sequences problem (solver algorithmn)
		 * @param dwl::planning::Solver *solver	Pointer to the solver of the motion planning algorithm
		 */
		void reset(Solver* solver);

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
		 * @param dwl::planning::BodyPose start Start pose
		 * @param dwl::planning::BodyPose goal Goal pose
		 */
		virtual void update(BodyPose start, BodyPose goal) = 0;

		/**
		 * @brief Computes the motion planning
		 * @return bool Return true if it was computed the plan
		 */
		bool computePlan();

		/**
		 * @brief Abstract method for the computation a motion plan according to added constraints and costs in the optimization problem
		 * @return bool Return true if it was found a plan
		 */
		virtual bool compute() = 0;

		/**
		 * @brief Changes the goal state
		 * @param std::vector<double> goal New goal state
		 */
		void changeGoal(BodyPose goal);

		/**
		 * @brief Gets the name of the planner
		 * @return std::string Return the name of the planner
		 */
		std::string getName();


	private:
		/** @brief Indicates if it was settep a solver algorithm for the computation of a plan */
		bool is_settep_solver_;

		/** @brief Indicates it was initialized the planning algorithm */
		bool is_initialized_planning_;

		/** @brief Initial pose of the robot */
		BodyPose initial_pose_;

		/** @brief Goal pose of the robot */
		BodyPose goal_pose_;

		//pthread_mutex_t planning_lock_;


	protected:
		/** @brief Name of the planner */
		std::string name_;

		/** @brief Pointer to the solver algorithm */
		Solver* solver_;

		/** @brief Vector of contact points */
		std::vector<Contact> contacts_sequence_;

		/** @brief Vector of body position */
		std::vector<BodyPose> pose_trajectory_;
};

} //@namespace planning

} //@namespace dwl


#endif
