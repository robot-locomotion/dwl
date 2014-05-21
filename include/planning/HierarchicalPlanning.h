#ifndef DWL_HierarchicalPlanning_H
#define DWL_HierarchicalPlanning_H


#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{

namespace planning
{

/**
 * @class HierarchicalPlanning
 * @brief Class for solving the problem of planning of motion sequences using a hierarchical approach
 */
class HierarchicalPlanning : public dwl::planning::PlanningOfMotionSequences
{
	public:
		/** @brief Constructor function */
		HierarchicalPlanning();

		/** @brief Destructor function */
		~HierarchicalPlanning() {}

		/**
		 * @brief Initializes the hierarchical planning given a start and goal state
		 * @param std::vector<double> start Start state
		 * @param std::vector<double> goal Goal state
		 */
		bool init(std::vector<double> start, std::vector<double> goal);

		/**
		 * @brief Computes a whole-body motion plan
		 */
		bool compute();

		//TODO: I think that for Hierarchical Planner we can implement a method for setting the reward map

	private:
		/** @brief Trajectory of body */
		std::vector<Eigen::Vector3d> body_path_;


}; //@class HierarchicalPlanning

} //@namespace dwl
}

#endif
