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

/**
 * @class ContactPlanner
 * @brief Class for computing contact plans
 */
class ContactPlanner
{
	public:
		/** @brief Constructor function */
		ContactPlanner();

		/** @brief Destructor function */
		~ContactPlanner();

		/**
		 * @brief Specifies the environment information for computing a contact plan
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(environment::EnvironmentInformation* environment);

		/**
		 * @bief Computes the footholds
		 * @param std::vector<Contact>& footholds Set of footholds
		 * @param dwl::Pose current_pose Current pose
		 */
		bool computeFootholds(std::vector<Contact>& footholds, Pose current_pose);


	protected:
		/** @brief Pointer to the environment information */
		environment::EnvironmentInformation* environment_;

		/** @brief Robot information */
		robot::Robot robot_;
};

} //@namespace planning
} //@namespace dwl

#endif
