#ifndef DWL_WholeBodyLocomotion_H
#define DWL_WholeBodyLocomotion_H

#include <locomotion/PlanningOfMotionSequences.h>


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
		 * @param dwl::locomotion::PlanningOfMotionSequences* planner Pointer to the planner
		 */
		void reset(locomotion::PlanningOfMotionSequences* planner);

		/**
		 * @brief Adds a constraint to the locomotor
		 * @param dwl::locomotion::Constraint* constraint Pointer to the constraint class
		 */
		void addConstraint(locomotion::Constraint* constraint);

		/**
		 * @brief Removes a constraint to the locomotor
		 * @param std::string constraint_name Name of the constraint
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost to the locomotor
		 * @param dwl::locomotion::Cost* cost Pointer to the cost class
		 */
		void addCost(locomotion::Cost* cost);

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
		 * @param dwl::Pose goal Goal pose
		 */
		void resetGoal(Pose goal);

		/**
		 * @brief Computes a locomotion plan
		 * @param dwl::Pose current_pose Current pose
		 * @return bool Return true if was found a locomotion plan
		 */
		bool compute(Pose current_pose);

		/**
		 * @brief Sets the reward map of the terrain
		 * @param std::vector<dwl::RewardCell> reward_map Reward map of the terrain
		 */
		void setTerrainInformation(std::vector<RewardCell> reward_map);

		/**
		 * @brief Sets the obstacle map of the terrain
		 * @param std::vector<dwl::Cell> obstacle_map Obstacle map of the terrain
		 */
		void setTerrainInformation(std::vector<Cell> obstacle_map);

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
		 * @return std::vector<dwl::Pose> Return the approximated body path
		 */
		std::vector<Pose> getBodyPath();

		std::vector<Contact> getContactSequence();


	private:
		/** @brief Pointer to the planner class */
		locomotion::PlanningOfMotionSequences* planner_;

		/** @brief Indicates if it was set the planner algorithm */
		bool is_set_planner_;

		/** @brief Indicates if it is using a learning technique */
		bool is_learning_;


	protected:
	
};

} //@namespace dwl


#endif
