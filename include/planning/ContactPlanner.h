#ifndef DWL_ContactPlanner_H
#define DWL_ContactPlanner_H

#include <environment/EnvironmentInformation.h>
#include <environment/Feature.h>
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
class ContactPlanner //TODO Convert this class to an abstract class
{
	public:
		/** @brief Constructor function */
		ContactPlanner();

		/** @brief Destructor function */
		~ContactPlanner();

		/**
		 * @brief Specifies the environment information for computing a contact plan
		 * @param dwl::robot::Robot* robot The robot defines all the properties of the robot
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, environment::EnvironmentInformation* environment);

		/**
		 * @brief Adds a feature for the contact planner
		 * @param dwl::environment::Feature* feature Feature
		 */
		void addFeature(environment::Feature* feature);

		/**
		 * @bief Computes the contacts given a current pose of the robot
		 * @param std::vector<Contact>& contacts Set of contacts
		 * @param std::vector<Contact> current_contacts Current contacts
		 * @param dwl::Pose goal_pose Goal pose
		 */
		bool computeContacts(std::vector<Contact>& contacts, std::vector<Contact> current_contacts, Pose goal_pose);

		/**
		 * @brief Sets the allowed computation time for the contact planner
		 * @param double computation_time Allowed computation time
		 */
		void setComputationTime(double computation_time);


	protected:
		/** @brief Pointer to the environment information */
		environment::EnvironmentInformation* environment_;

		/** @brief Pointer to the robot properties information */
		robot::Robot* robot_;

		/** @brief Vector of features */
		std::vector<environment::Feature*> features_;

		/** @brief Allowed computation time */
		double computation_time_;

		double leg_offset_;
};

} //@namespace planning
} //@namespace dwl

#endif
