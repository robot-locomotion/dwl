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

class BodyPlanner
{
	public:
		BodyPlanner();
		~BodyPlanner();

		/**
		 * @brief Specifies the environment information
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(environment::EnvironmentInformation* environment);

		void reset(Solver* path_planner);

		bool computeBodyPath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose);

		//void findPose();

	protected:
		environment::EnvironmentInformation* environment_;

		Solver* path_solver_;

		Solver* pose_solver_;
};

} //@namespace planning
} //@namespace dwl

#endif
