#ifndef DWL__MODEL__CONSTRAINED_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__CONSTRAINED_DYNAMICAL_SYSTEM__H

#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

/**
 * @class ConstrainedDynamicalSystem
 * @brief ConstrainedDynamicalSystem class defines an constrained inverse dynamic constraint. This
 * inverses dynamic algorithm assumes a constrained set of contacts (active contacts), and maps
 * the desired floating-base motion to required contact forces. Additionally, active contact
 * velocities are constrained to be zero, which corresponds to predefined assumptions. The
 * constraint dimension is: [number of joints + 3 * (number of active contacts)]
 */
class ConstrainedDynamicalSystem : public DynamicalSystem
{
	public:
		/** @brief Constructor function */
		ConstrainedDynamicalSystem();

		/** @brief Destructor function */
		~ConstrainedDynamicalSystem();

		/**
		 * @brief Sets the active end-effectors, i.e. end-effectors in contact
		 * @param const rbd::BodySelector& Set of active end-effectors
		 */
		void setActiveEndEffectors(const rbd::BodySelector& active_set);

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
		/** @brief Set of active end-effectors */
		rbd::BodySelector active_endeffectors_;
};

} //@namespace model
} //@namespace dwl

#endif
