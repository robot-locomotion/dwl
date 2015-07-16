#ifndef DWL__MODEL__FULL_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__FULL_DYNAMICAL_SYSTEM__H

#include <model/DynamicalSystem.h>
#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

class FullDynamicalSystem : public DynamicalSystem
{
	public:
		/** @brief Constructor function */
		FullDynamicalSystem();

		/** @brief Destructor function */
		~FullDynamicalSystem();

		/**
		 * @brief Computes the dynamic constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		void compute(Eigen::VectorXd& constraint,
					 const LocomotionState& state);

		/**
		 * @brief Computes the Jacobian of the full dynamic constraint given a certain state
		 * @param Eigen::MatrixXd& Jacobian of the constraint function
		 * @param const LocomotionState& State vector
		 */
		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const LocomotionState& state);

		/**
		 * @brief Gets the bounds of the dynamical system constraint
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);


	private:
		WholeBodyDynamics dynamics_;
};

} //@namespace model
} //@namespace dwl

#endif
