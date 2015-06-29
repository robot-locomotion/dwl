#include <model/Constraint.h>
#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

class FullDynamicSystem : public Constraint
{
	public:
		/** @brief Constructor function */
		FullDynamicSystem();

		/** @brief Destructor function */
		~FullDynamicSystem();

		/**
		 * @brief Computes the full dynamic constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const StateModel& State vector
		 */
		void compute(Eigen::VectorXd& constraint, const StateModel& state);

		/**
		 * @brief Computes the Jacobian of the full dynamic constraint given a certain state
		 * @param Eigen::MatrixXd& Jacobian of the constraint function
		 * @param const StateModel& State vector
		 */
		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const StateModel& state);
};

} //@namespace model
} //@namespace dwl
