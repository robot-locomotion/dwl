#ifndef DWL__MODEL__LINEAR_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__LINEAR_DYNAMICAL_SYSTEM__H

#include <dwl/model/DynamicalSystem.h>
#include <Eigen/Dense>


namespace dwl
{

namespace model
{
/**
 * @brief This is the abstract class used to create and define different process models, in a
 * state-space representation. The models can be defined as Linear Time Invariant (LTI) such as
 * \f{eqnarray*}{
 *	\dot{x}(t) = Ax(t) + Bu(t) \\
 *	y(t) = Cx(t)
 * \f}
 * or Linear Time Variant (LTV) such as
 * \f{eqnarray*}{
 *	\dot{x}(t) = A(t)x(t) + B(t)u(t) \\
 *	y(t) = C(t)x(t)
 * \f}
 * A boolean member variable defines the type of model used. The C matrix is not considered in the
 * computation of the system matrices because this matrix is not used by the MPC algorithm, in an
 *  effort to reduce computation time.
 */
class LinearDynamicalSystem : public DynamicalSystem
{
	public:
       	/** @brief Constructor function	*/
		LinearDynamicalSystem();

		/** @brief Destructor function */
		virtual ~LinearDynamicalSystem();

		/**
		 * @brief After the MPC makes an iteration, this function is used to set the current state
		 * as the new linearization points for a LTV model into global variables.
		 * @param const Eigen::VectorXd& New linearization point for the state vector
		 */
		virtual void setLinearizationPoints(const Eigen::VectorXd& op_states) = 0;

		/**
		 * @brief Function to compute the matrices for a Linear model process. If the model is a
		 * LTI model, the function can be just defined to set the values of the model matrices.
		 * @param Eigen::MatrixXd& State or System matrix
		 * @param Eigen::MatrixXd& Input matrix
		 * @return bool Label that indicates if the computation of the matrices is successful
		 */
		virtual void computeLinearSystem(Eigen::MatrixXd& A, Eigen::MatrixXd& B) = 0;

		/** @brief Get the inputs number of the dynamic model */
		virtual int getInputsNumber() const;

		/** @brief Get the outputs number of the dynamic model */
		virtual int getOutputsNumber() const;

		/**
		 * @brief Function to identify if the model is LTI or LTV
		 * @return bool True if the model is LTV
		 */
		virtual bool getModelType() const;

		/** @brief Function that returns the current value of the operation points for the states */
		virtual double* getOperationPointsStates() const;

		/** @brief Function that returns the current value of the operation points for the inputs */
		virtual double* getOperationPointsInputs() const;


	protected:
		/** @brief State matrix of the dynamic model */
		Eigen::MatrixXd A_;

		/** @brief Input matrix of the dynamic model */
		Eigen::MatrixXd B_;

		/** @brief Number of inputs of the dynamic model */
		unsigned int num_inputs_;

		/** @brief Number of outputs of the dynamic model */
		unsigned int num_outputs_;

		/** @brief Operation state **/
		Eigen::VectorXd op_point_states_;

		/** @brief Operation input **/
		Eigen::VectorXd op_point_input_;

		/** @brief Boolean to check if the model is time variant or not (True = LTV) */
		bool time_variant_;
};

} //@namespace model
} //@namespace dwl


inline int dwl::model::LinearDynamicalSystem::getInputsNumber() const
{
	return num_inputs_;
}

inline int dwl::model::LinearDynamicalSystem::getOutputsNumber() const
{
	return num_outputs_;
}

inline bool dwl::model::LinearDynamicalSystem::getModelType() const
{	
	return time_variant_;
}

inline double* dwl::model::LinearDynamicalSystem::getOperationPointsStates() const
{
	return op_point_states_;
}

inline double* dwl::model::LinearDynamicalSystem::getOperationPointsInputs() const
{
	return op_point_input_;
}

#endif

