#ifndef DWL__MODEL__CENTROIDAL_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__CENTROIDAL_DYNAMICAL_SYSTEM__H

#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

/**
 * @class CentroidalDynamicalSystem
 * @brief CentroidalDynamicalSystem class describes the dynamical system by using centroidal
 * dynamics representation. The centroidal dynamics describes the dynamics of the robot projected
 * at its CoM. CoM dynamics are decomposed in two equations: linear momentum, and angular momentum.
 * For more information read Orin et al. "Centroidal dynamics of a humanoid robot". An additional
 * constraint is imposed in order to model the nature of the contact position, i.e. the contact
 * position is a decision variable.
 */
class CentroidalDynamicalSystem : public DynamicalSystem
{
	public:
		/** @brief Constructor function */
		CentroidalDynamicalSystem();

		/** @brief Destructor function */
		~CentroidalDynamicalSystem();

		/** @brief Initializes the centroidal dynamical system constraint */
		void initDynamicalSystem();

		/**
		 * @brief Computes the centroidal dynamics constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		void computeDynamicalConstraint(Eigen::VectorXd& constraint,
										const LocomotionState& state);

		/**
		 * @brief Gets the bounds of the centroidal dynamical system constraint
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getDynamicalBounds(Eigen::VectorXd& lower_bound,
								Eigen::VectorXd& upper_bound);

	private:
		/** @brief End-effector names */
		std::vector<std::string> end_effector_names_;

		/** @brief Total mass */
		double total_mass_;
};

} //@namespace model
} //@namespace dwl

#endif
