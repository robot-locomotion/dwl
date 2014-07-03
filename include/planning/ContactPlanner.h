#ifndef DWL_ContactPlanner_H
#define DWL_ContactPlanner_H

#include <environment/EnvironmentInformation.h>
#include <robot/Robot.h>
#include <utils/utils.h>
#include <utils/Orientation.h>


namespace dwl
{

namespace planning
{

class ContactPlanner
{
	public:
		ContactPlanner();
		~ContactPlanner();

		void reset(environment::EnvironmentInformation* environment);

		bool computeFootholds(std::vector<Contact>& footholds, Pose current_pose);


	protected:
		environment::EnvironmentInformation* environment_;

		robot::Robot robot_;
};

} //@namespace planning
} //@namespace dwl

#endif
