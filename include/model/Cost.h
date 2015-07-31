#ifndef DWL__MODEL__COST__H
#define DWL__MODEL__COST__H

#include <environment/RewardMap.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class Cost
 * @brief Abstract class for computing the cost function in optimization-based locomotion (planning
 * or control) approach
 */
class Cost
{
	public:
		/** @brief Constructor function */
		Cost();

		/** @brief Destructor function */
		virtual ~Cost();

		/**
		 * @brief Computes the cost value given a certain state
		 * @param double& Cost value
		 * @param const LocomotionState& State value
		 */
		virtual void compute(double& cost,
							 const LocomotionState& state) = 0;

		/**
		 * @brief Sets the locomotion state weights which are used by specific cost function
		 * @param LocomotionState& weights
		 */
		void setWeights(LocomotionState& weights);

		/**
		 * @brief Gets the name of the cost
		 * @return The name of the cost
		 */
		std::string getName();


	protected:
		/** @brief Name of the cost */
		std::string name_;

		/** @brief Locomotion state weights */
		LocomotionState locomotion_weights_;
};

} //@namespace model
} //@namespace dwl

#endif
