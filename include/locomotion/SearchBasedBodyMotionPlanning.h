#ifndef DWL__LOCOMOTION__SEARCH_BASED_BODY_MOTION_PLANNING__H
#define DWL__LOCOMOTION__SEARCH_BASED_BODY_MOTION_PLANNING__H

#include <locomotion/MotionPlanning.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class SearchBasedBodyMotionPlanning
 * @brief SearchBasedBodyMotionPlanning is derived from MotionPlanning and computes the body path
 * based on search algorithm such as A*, ARA*, etc.
 */
class SearchBasedBodyMotionPlanning : public MotionPlanning
{
	public:
		/** @brief Constructor function */
		SearchBasedBodyMotionPlanning();

		/** @brief Destructor function */
		~SearchBasedBodyMotionPlanning();

		/**
		 * @brief Computes the body path from start pose to goal pose
		 * @param std::vector<Pose>& Planned body path
		 * @param Pose Start pose
		 * @param Pose Goal pose
		 */
		bool computePath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose);
};

} //@namespace locomotion
} //@namespace dwl

#endif
