#ifndef DWL__MODEL__INELASTIC_CONTACT_MODEL_CONSTRAINT__H
#define DWL__MODEL__INELASTIC_CONTACT_MODEL_CONSTRAINT__H

#include <model/ComplementaryConstraint.h>


namespace dwl
{

namespace model
{

class InelasticContactModelConstraint : public ComplementaryConstraint
{
	public:
		/** @brief Constructor function */
		InelasticContactModelConstraint();

		/** @brief Destructor function */
		~InelasticContactModelConstraint();

		/**
		 * @brief Computes the first complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		void computeFirstComplement(Eigen::VectorXd& constraint,
									const LocomotionState& state);

		/**
		 * @brief Computes the second complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		void computeSecondComplement(Eigen::VectorXd& constraint,
									 const LocomotionState& state);
};

} //@namespace model
} //@namespace dwl

#endif
