#ifndef DWL__MODEL__COMPLEMENTARY_CONSTRAINT__H
#define DWL__MODEL__COMPLEMENTARY_CONSTRAINT__H

#include <model/Constraint.h>


namespace dwl
{

namespace model
{

class ComplementaryConstraint : public Constraint
{
	public:
		/** @brief Constructor function */
		ComplementaryConstraint();

		/** @brief Destructor function */
		virtual ~ComplementaryConstraint();

		/**
		 * @brief Computes the complementary constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		void compute(Eigen::VectorXd& constraint,
					 const LocomotionState& state);

		/**
		 * @brief Gets the bounds of the complementary constraints
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);

		/**
		 * @brief Computes the first complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void computeFirstComplement(Eigen::VectorXd& constraint,
											const LocomotionState& state) = 0;

		/**
		 * @brief Computes the second complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void computeSecondComplement(Eigen::VectorXd& constraint,
											 const LocomotionState& state) = 0;


	private:
		unsigned int complementary_dimension_;
};

} //@namespace model
} //@namespace dwl

#endif
