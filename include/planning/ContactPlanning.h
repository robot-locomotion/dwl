#ifndef DWL_ContactPlanning_H
#define DWL_ContactPlanning_H

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
 * @class ContactPlanning
 * @brief Abstract class for computing contact sequence
 */
class ContactPlanning
{
	public:
		/** @brief Constructor function */
		ContactPlanning();

		/** @brief Destructor function */
		virtual ~ContactPlanning();

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
		 * @param std::vector<Contact>& contact_sequence Set of contacts
		 * @param std::vector<dwl::Pose> pose_trajectory Goal pose
		 */
		virtual bool computeContactSequence(std::vector<Contact>& contact_sequence, std::vector<Pose> pose_trajectory) = 0;

		/**
		 * @brief Sets the allowed computation time for the contact planner
		 * @param double computation_time Allowed computation time
		 */
		void setComputationTime(double computation_time);

		/**
		 * @brief Sets the contact horizon, number of contacts, of the planner
		 * @param int horizon Number of contacts
		 */
		void setContactHorizon(int horizon);


	protected:
		/** @brief Pointer to the environment information */
		environment::EnvironmentInformation* environment_;

		/** @brief Pointer to the robot properties information */
		robot::Robot* robot_;

		/** @brief Vector of features */
		std::vector<environment::Feature*> features_;

		/** @brief Allowed computation time */
		double computation_time_;

		/** @brief Current body state */
		Eigen::Vector3d current_body_state_;

		/** @brief Contact horizon */
		double contact_horizon_;
};

} //@namespace planning
} //@namespace dwl

#endif
