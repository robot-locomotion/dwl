#ifndef DWL_KinematicFeasibilityFeature_H
#define DWL_KinematicFeasibilityFeature_H

#include <environment/Feature.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class KinematicFeasibilityFeature
 * @brief Class for computing the reward value of the kinematic feasibility feature
 */
class KinematicFeasibilityFeature : public Feature
{
	public:
		/** @brief Constructor function */
		KinematicFeasibilityFeature(double kin_lim_x, double kin_lin_y, double stable_displacement);

		/** @brief Destructor function */
		~KinematicFeasibilityFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& Reward value
		 * @param RobotAndTerrain Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:
		/** @brief Maximum allowed displacement in x */
		double kin_lim_x_;

		/** @brief Maximum allowed displacement in y */
		double kin_lim_y_;

		/** @brief Stable displacement */
		double stable_displacement_;
};

} //@namespace environment
} //@namespace dwl

#endif
