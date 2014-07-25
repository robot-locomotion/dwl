#ifndef DWL_BodyPlanner_H
#define DWL_BodyPlanner_H

#include <planning/Solver.h>
#include <environment/EnvironmentInformation.h>
#include <utils/utils.h>
#include <utils/Orientation.h>


namespace dwl
{


namespace planning
{

/**
 * @class BodyPlanner
 * @brief Class for implementing a body planner
 */
class BodyPlanner
{
	public:
		/** @brief Constructor function */
		BodyPlanner();

		/** @brief Destructor function */
		~BodyPlanner();

		/**
		 * @brief Specifies the environment information
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(environment::EnvironmentInformation* environment);

		void reset(Solver* path_planner);

		/**
		 * @brief Computes the body path from goal pose
		 * @param std::vector<dwl::Pose>& body_path Body path
		 * @param dwl::Pose start_pose Start pose
		 * @param dwl::Pose goal_pose Goal pose
		 */
		bool computeBodyPath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose);

		//void findPose();

	protected:
		/** @brief Pointer to the EnvironmentInformation object */
		environment::EnvironmentInformation* environment_;

		/** @brief Pointer to the path solver */
		Solver* path_solver_;

		/** @brief Pointer to the pose solver */
		Solver* pose_solver_;
};

} //@namespace planning
} //@namespace dwl

#endif
