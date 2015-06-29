#ifndef DWL_Constraint_H
#define DWL_Constraint_H

#include <utils/utils.h>


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
		 * @param const StateModel& State vector
		 */
		virtual void compute(Eigen::VectorXd& constraint,
							 const StateModel& state) = 0;

		/**
		 * @brief Computes the Jacobian of the constraint given a certain state
		 * @param Eigen::MatrixXd& Jacobian of the constraint function
		 * @param const StateModel& State vector
		 */
		virtual void computeJacobian(Eigen::MatrixXd& jacobian,
									 const StateModel& state) = 0;

		int getConstraintDimension();

		/**
		 * @brief Indicates if the constraint is active [g(x) = 0] or inactive [g(x) > 0]
		 * @return True for active constraints, and false for inactive one
		 */
		bool isActive();

		/**
		 * @brief Gets the name of the constraint
		 * @return The name of the constraint
		 */
		std::string getName();


	protected:
		/** @brief Name of the constraint */
		std::string name_;

		/** @brief Indicates if the constraint is active */
		bool is_active_constraint_;

		int constraint_dimension_;

		/** @brief Vector of the state */
		Eigen::VectorXd state_value_;

		/** @brief Vector of the values of the constraint */
		Eigen::VectorXd constraint_value_;
};

} //@namespace model
} //@namespace dwl

#endif
