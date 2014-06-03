#ifndef DWL_PlanningOfMotionSequences_H
#define DWL_PlanningOfMotionSequences_H

#include <planning/Solver.h>
#include <planning/Constraint.h>
#include <planning/Cost.h>
#include <environment/PlaneGrid.h>

#include <Eigen/Dense>
#include <vector>

#include <utils/utils.h>

//#include <pthread.h>


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
		 * @param dwl::planning::Pose start Start pose
		 * @param dwl::planning::Pose goal Goal pose
		 */
		virtual void update(Pose start, Pose goal) = 0;

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
		 * @brief Changes the goal pose
		 * @param dwl::planning::Pose goal New goal pose
		 */
		void changeGoal(Pose goal);

		/**
		 * @brief Sets the gridmap resolution
		 * @param double resolution Resolution of the gridmap
		 */
		void setGridMapResolution(double resolution);

		/**
		 * @brief Gets the approximated body path
		 * @return std::vector<Pose> Return the approximated body path
		 */
		std::vector<Pose> getBodyPath();

		/**
		 * @brief Gets the name of the planner
		 * @return std::string Return the name of the planner
		 */
		std::string getName();


	private:
		/** @brief Initial pose of the robot */
		Pose initial_pose_;

		/** @brief Goal pose of the robot */
		Pose goal_pose_;

		/** @brief Indicates if it was settep a solver algorithm for the computation of a plan */
		bool is_settep_solver_;

		/** @brief Indicates it was initialized the planning algorithm */
		bool is_initialized_planning_;

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_active_constraint_;

		/** @brief Indicates if it was added an inactive constraint in the solver */
		bool is_added_inactive_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;

		//pthread_mutex_t planning_lock_;


	protected:
		/** @brief Name of the planner */
		std::string name_;

		/** @brief Pointer to the solver algorithm */
		Solver* solver_;

		/** @brief Vector of active constraints pointers */
		std::vector<Constraint*> active_constraints_;

		/** @brief Vector of inactive constraints pointers */
		std::vector<Constraint*> inactive_constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		/** @brief Gridmap representation */
		environment::PlaneGrid gridmap_;

		/** @brief Vector of contact points */
		std::vector<Contact> contacts_sequence_;

		/** @brief Vector of body position */
		std::vector<Pose> body_trajectory_;
};

} //@namespace planning

} //@namespace dwl


#endif
