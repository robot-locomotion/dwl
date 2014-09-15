#include <planning/MotionPlanning.h>


namespace dwl
{

namespace planning
{

class SearchBasedBodyMotionPlanning : public MotionPlanning
{
	public:
		/** @brief Constructor function */
		SearchBasedBodyMotionPlanning();

		/** @brief Destructor function */
		~SearchBasedBodyMotionPlanning();

		/**
		 * @brief Computes the body path from start pose to goal pose
		 * @param std::vector<dwl::Pose>& body_path Body path
		 * @param dwl::Pose start_pose Start pose
		 * @param dwl::Pose goal_pose Goal pose
		 */
		bool computePath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose);
};

} //@namespace planning
} //@namespace dwl
