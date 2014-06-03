#ifndef DWL_WholeBodyLocomotion_H
#define DWL_WholeBodyLocomotion_H

#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{

/**
 * @class WholeBodyLocomotion
 * @brief Class for solving the whole-body locomotion problem
 */
class WholeBodyLocomotion
{

	public:
		/** @brief Constructor function */
		WholeBodyLocomotion();

		/** @brief Destructor function */
		~WholeBodyLocomotion();

		/**
		 * @brief Resets the planning of motion sequences algorithm
		 * @param dwl::planning::PlanningOfMotionSequences* planner Pointer to the planner
		 */
		void reset(planning::PlanningOfMotionSequences* planner);

		/**
		 * @brief Adds a constraint to the locomotor
		 * @param dwl::planning::Constraint* constraint Pointer to the constraint class
		 */
		void addConstraint(planning::Constraint* constraint);

		/**
		 * @brief Removes a constraint to the locomotor
		 * @param std::string constraint_name Name of the constraint
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost to the locomotor
		 * @param dwl::planning::Cost* cost Pointer to the cost class
		 */
		void addCost(planning::Cost* cost);

		/**
		 * @brief Removes a cost to the locomotor
		 * @param std::string cost_name Name of the cost
		 */
		void removeCost(std::string cost_name);

		/**
		 * @brief Initializes a locomotion algorithm
		 */
		bool init();

		/**
		 * @brief Updates the start and goal pose of the robot for making a receding horizon planning
		 * @param dwl::Pose start Start pose
		 * @param dwl::Pose goal Goal pose
		 */
		void update(Pose start, Pose goal);

		/**
		 * @brief Computes a locomotion plan
		 * @param dwl::Pose robot_state Robot pose
		 * @return bool Return true if was found a locomotion plan
		 */
		bool compute(Pose robot_state);

		/**
		 * @brief Sets the gridmap resolution
		 * @param double resolution Resolution of the gridmap
		 */
		void setGridMapResolution(double resolution);

		/**
		 * @brief Changes the goal
		 * @param dwl::Pose goal Goal state
		 */
		void changeGoal(Pose goal);

		/**
		 * @brief Gets the approximated body path
		 * @return std::vector<dwl::Pose> Return the approximated body path
		 */
		std::vector<Pose> getBodyPath();


	private:
		/** @brief Pointer to the planner class */
		planning::PlanningOfMotionSequences* planner_;

		/** @brief Indicates if it was settep the planner algorithm */
		bool is_settep_planner_;

		/** @brief Indicates if it is using a lerning technique */
		bool is_learning_;


	protected:
	
	
}; //@class WholeBodyLocomotion

} //@namespace dwl


#endif
