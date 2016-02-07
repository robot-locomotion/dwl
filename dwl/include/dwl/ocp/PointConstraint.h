#ifndef DWL__OCP__POINT_CONSTRAINT__H
#define DWL__OCP__POINT_CONSTRAINT__H

#include <dwl/ocp/Constraint.h>


namespace dwl
{

namespace ocp
{

class PointConstraint : public Constraint<Eigen::VectorXd>
{
	public:
		/** @brief Constructor functions */
		PointConstraint();
		PointConstraint(Eigen::VectorXd& lower_bound,
						Eigen::VectorXd& upper_bound);

		/** @brief Destructor function */
		~PointConstraint();

		/**
		 * @brief Computes the constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const Eigen::VectorXd& Point state
		 */
		void compute(Eigen::VectorXd& constraint,
					 const Eigen::VectorXd& state);

		/**
		 * @brief Gets the lower and upper bounds of the constraint
		 * @param Eigen::VectorXd& Lower constraint bound
		 * @param Eigen::VectorXd& Upper constraint bound
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);


	private:
		/** @brief Lower bound */
		Eigen::VectorXd lower_bound_;

		/** @brief Upper bound */
		Eigen::VectorXd upper_bound_;
};

} //@namespace ocp
} //@namespace dwl

#endif
