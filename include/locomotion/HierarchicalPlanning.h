#ifndef DWL_HierarchicalPlanning_H
#define DWL_HierarchicalPlanning_H

#include <locomotion/PlanningOfMotionSequence.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class HierarchicalPlanning
 * @brief Class for solving the problem of planning of motion sequences using a hierarchical approach
 */
class HierarchicalPlanning : public PlanningOfMotionSequence
{
	public:
		/** @brief Constructor function */
		HierarchicalPlanning();

		/** @brief Destructor function */
		~HierarchicalPlanning();

		/**
		 * @brief Initializes of the hierarchical planning
		 * @return bool Return true if was initialized the hierarchical planning
		 */
		virtual bool init();

		/**
		 * @brief Updates the goal body pose for hierarchical planning
		 * @param dwl::Pose goal Goal pose
		 */
		virtual void resetGoal(Pose goal);

		/**
		 * @brief Computes a whole-body motion provided for the hierarchical planning
		 * @param dwl::Pose current_pose Current pose
		 * @return bool Return true if it was computed the plan
		 */
		bool compute(Pose current_pose);


	private:

};

} //@namespace locomotion
} //@namespace dwl


#endif
