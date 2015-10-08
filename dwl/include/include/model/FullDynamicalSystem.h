#ifndef DWL__MODEL__FULL_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__FULL_DYNAMICAL_SYSTEM__H

#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

/**
 * @class FullDynamicalSystem
 * @brief FullDynamicalSystem class describes the dynamical system using a full inverse
 * dynamics algorithm. An additional constraint is imposed it in order to model the nature of
 * floating-base system. i.e. base wrench equals to zero.
 */
class FullDynamicalSystem : public DynamicalSystem
{
	public:
		/** @brief Constructor function */
		FullDynamicalSystem();

		/** @brief Destructor function */
		~FullDynamicalSystem();

		/** @brief Initializes the full dynamical system constraint */
		void initDynamicalSystem();

		/**
		 * @brief Computes the dynamic constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		void computeDynamicalConstraint(Eigen::VectorXd& constraint,
										const LocomotionState& state);

		/**
		 * @brief Gets the bounds of the dynamical system constraint
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getDynamicalBounds(Eigen::VectorXd& lower_bound,
								Eigen::VectorXd& upper_bound);


	private:
		/** @brief End-effector names */
		std::vector<std::string> end_effector_names_;
};

} //@namespace model
} //@namespace dwl

#endif
