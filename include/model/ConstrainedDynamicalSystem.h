#ifndef DWL__MODEL__CONSTRAINED_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__CONSTRAINED_DYNAMICAL_SYSTEM__H

#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

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
		void compute(Eigen::VectorXd& constraint,
					 const LocomotionState& state);

		/** @brief Gets the constraint dimension of the constrained dynamical system */
		unsigned int getConstraintDimension();

		/**
		 * @brief Gets the bounds of the dynamical system constraint
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);


	private:
		/** @brief Set of active end-effectors */
		rbd::BodySelector active_endeffectors_;
};

} //@namespace model
} //@namespace dwl

#endif
