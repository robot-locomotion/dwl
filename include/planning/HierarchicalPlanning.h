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
		~HierarchicalPlanning();

		/**
		 * @brief Initializes of the hierarchical planning
		 * @return bool Return true if was initialized the hierarchical planning
		 */
		virtual bool init();

		/**
		 * @brief Updates the start and goal body pose for hierarchical planning
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
		/**
		 * @brief Converts the pose to a 3d state (x,y,yaw)
		 * @param Eigen::Vector3d& state State
		 * @param dwl::Pose pose Pose
		 */
		void poseToState(Eigen::Vector3d& state, Pose pose);

}; //@class HierarchicalPlanning


} //@namespace planning
} //@namespace dwl


#endif
