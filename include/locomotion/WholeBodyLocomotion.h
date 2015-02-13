#ifndef DWL_WholeBodyLocomotion_H
#define DWL_WholeBodyLocomotion_H

#include <locomotion/PlanningOfMotionSequence.h>


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
		 * @param PlanningOfMotionSequences* Pointer to the planner
		 */
		void reset(locomotion::PlanningOfMotionSequence* planner);

		/**
		 * @brief Adds a constraint to the locomotion approach
		 * @param Constraint* Pointer to the constraint class
		 */
		void addConstraint(constraint::Constraint* constraint);

		/**
		 * @brief Removes a constraint to the locomotion approach
		 * @param std::string Name of the constraint
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost to the locomotion approach
		 * @param Cost* Pointer to the cost class
		 */
		void addCost(locomotion::Cost* cost);

		/**
		 * @brief Removes a cost to the locomotion approach
		 * @param std::string Name of the cost
		 */
		void removeCost(std::string cost_name);

		/**
		 * @brief Initializes a locomotion algorithm
		 */
		bool init();

		/**
		 * @brief Updates the start and goal pose of the robot for making a receding horizon planning
		 * @param Pose Goal pose
		 */
		void resetGoal(Pose goal);

		/**
		 * @brief Computes a locomotion plan
		 * @param Pose Current pose
		 * @return True if was found a locomotion plan
		 */
		bool compute(Pose current_pose);

		/**
		 * @brief Sets the reward map of the terrain
		 * @param std::vector<RewardCell> Reward map of the terrain
		 */
		void setTerrainInformation(std::vector<RewardCell> reward_map);

		/**
		 * @brief Sets the obstacle map of the terrain
		 * @param std::vector<Cell> Obstacle map of the terrain
		 */
		void setTerrainInformation(std::vector<Cell> obstacle_map);

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


	private:
		/** @brief Pointer to the planner class */
		locomotion::PlanningOfMotionSequence* planner_;

		/** @brief Indicates if it was set the planner algorithm */
		bool is_set_planner_;

		/** @brief Indicates if it is using a learning technique */
		bool is_learning_;
};

} //@namespace dwl


#endif
