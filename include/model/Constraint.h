#ifndef DWL__MODEL__CONSTRAINT__H
#define DWL__MODEL__CONSTRAINT__H

#include <utils/utils.h>
#define NO_BOUND 2e19


namespace dwl
{

namespace model
{

/**
 * @class Constraint
 * @brief Abstract class for defining constraints in the planning of motion sequences problem
 */
class Constraint
{
	public:
		/** @brief Constructor function */
		Constraint();

		/** @brief Destructor function */
		virtual ~Constraint();

		/**
		 * @brief Computes the constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void compute(Eigen::VectorXd& constraint,
							 const LocomotionState& state) = 0;

		/**
		 * @brief Computes the Jacobian of the constraint given a certain state
		 * @param Eigen::MatrixXd& Jacobian of the constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void computeJacobian(Eigen::MatrixXd& jacobian,
									 const LocomotionState& state) = 0;

		virtual void getBounds(Eigen::VectorXd& lower_bound,
							   Eigen::VectorXd& upper_bound) = 0;

		/** @brief Gets the dimension of the constraint */
		unsigned int getConstraintDimension();

		/**
		 * @brief Gets the name of the constraint
		 * @return The name of the constraint
		 */
		std::string getName();


	protected:
		/** @brief Name of the constraint */
		std::string name_;

		/** @brief Dimension of the constraint */
		unsigned int constraint_dimension_;

		/** @brief Vector of the state */
		Eigen::VectorXd state_value_;

		/** @brief Vector of the values of the constraint */
		Eigen::VectorXd constraint_value_;
};

} //@namespace model
} //@namespace dwl

#endif
