#ifndef DWL_Constraint_H
#define DWL_Constraint_H

#include <Eigen/Dense>
//#include <iostream>
#include <utils/macros.h>


namespace dwl
{

namespace planning
{

/**
 * @class Constraint
 * @brief Abstract class for defining constraints in the planning of motion sequences problem (optimization problem)
 */
class Constraint
{
	public:
		/** @brief Constructor function */
		Constraint() : is_active_constraint_(false) {}

		/** @brief Destructor function */
		virtual ~Constraint() {}

		/**
		 * @brief Abstract method for getting the constraint vector given a certain state
		 * @param Eigen::VectorXd& constraint Constraint vector
		 * @param Eigen::VectorXd state State vector
		 */
		virtual void get(Eigen::VectorXd& constraint, Eigen::VectorXd state) = 0;

		/**
		 * @brief Indicates if the constraint is active [g(x) = 0] or inactive [g(x) > 0]
		 * @return Bool Return true for active constraints, and false for inactive one
		 */
		bool isActive();

		/**
		 * @brief Gets the name of the constraint
		 * @return std::string Retunr the name of the constraint
		 */
		std::string getName();

	protected:
		/** @brief Name of the constraint */
		std::string name_;

		/** @brief Indicates if the constraint is active */
		bool is_active_constraint_;

		/** @brief Vector of the state */
		Eigen::VectorXd state_value_;

		/** @brief Vector of the values of the constraint */
		Eigen::VectorXd constraint_value_;


}; //@class Constraint


} //@namespace planning

} //@namespace dwl




#endif
