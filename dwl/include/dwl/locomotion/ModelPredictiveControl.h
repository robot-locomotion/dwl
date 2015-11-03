#ifndef DWL__LOCOMOTION__MODEL_PREDICTIVE_CONTROL__H
#define DWL__LOCOMOTION__MODEL_PREDICTIVE_CONTROL__H

#include <dwl/model/LinearDynamicalSystem.h>
#include <dwl/solver/QuadraticProgram.h>

#include <Eigen/Dense>

#include <fstream>


namespace dwl
{

namespace locomotion
{

/**
 @class ModelPredictiveControl
 @brief This class serves as a base class in order to expand the functionality of the library and
 implement different sorts of MPC algorithms. The methods defined here are conceived in the simplest
 way possible to allow different implementations in the derived classes.\n
*/
class ModelPredictiveControl
{
	public:
		/** @brief Constructor function */
		ModelPredictiveControl();

		/** @brief Destructor function */
		~ModelPredictiveControl();

		/**
		 @brief Function to specify and set the settings of all the components within the MPC
		 problem. The dwl::locomotion::ModelPredictiveControl class can change individual parts of
		 the MPC problem; such as the model (dwl::model::LinearDynamicalSystem and derived classes)
		 and the optimizer (dwl::solver::QuadracticProgram and derived classes)
		 @param dwl::model::LinearDynamicalSystem Pointer to the model of the plant to be used in
		 the algorithm
		 @param dwl::solver::QuadraticProgram Pointer to the optimization library to be used in the
		 algorithm
		 */
		virtual bool reset(dwl::model::LinearDynamicalSystem* model,
						   dwl::solver::QuadraticProgram* optimizer) = 0;

		/**
		 @brief Function to initialize the calculation of the MPC algorithm. The function reads
		 all required parameters from ROS' parameter server that has been previously loaded from
		 a configuration YAML file, and performs all the initial calculations of variables to be
		 used in the optimization problem.
		 @return Label that indicates if the MPC is initialized with success
		 */
		virtual bool init() = 0;

		/**
		  @brief Function to update the MPC algorithm for the next iteration. The parameters
		  defined and calculated in dwl::solver::ModelPredictiveControl::init() are used together
		  with the methods taken from the MPC class components (dwl::model::LinearDynamicalSystem,
		  and dwl::solver::QuadraticProgram) to find a solution to the optimization problem.
		  This is where the different variants of MPC algorithms can be implemented in a source
		  file from a derived class.
		  @param double* State vector
		  @param double* Reference vector
		 */
		virtual void update(double* x_measured,
							double* x_reference) = 0;
			
		/**
		 @brief Function to get the control signal generated for the MPC. As the MPC algorithm
		 states, the optimization process yields the control signals for a range of times defined
		 by the prediction horizon, but only the current control signal is applied to the plant.
		 This function returns the control signal for the current time.
		 @return double* Control signal for the current MPC iteration.
		 */
		virtual double* getControlSignal() const;

			
	protected:
		/** @brief Pointer of linear dynamical model of the system */
		dwl::model::LinearDynamicalSystem* model_;
			
		/** @brief Pointer of the optimizer of the MPC */
		dwl::solver::QuadraticProgram* optimizer_;
			
		/** @brief Number of states of the dynamic model */
		int states_;
			
		/** @brief Number of inputs of the dynamic model */
		int inputs_;
			
		/** @brief Number of outputs of the dynamic model */
		int outputs_;
			
		/** @brief Horizon of prediction of the dynamic model */
		int horizon_;
			
		/** @brief Number of variables, i.e inputs * horizon */
		int variables_;
			
		/** @brief Number of constraints */
		int constraints_;

		/** @brief Vector of the operation points for the states in case of a LTI model */
		double* operation_states_;

		/** @brief Vector of the operation points for the inputs in case of a LTI model */
		double* operation_inputs_;
			
		/** @brief Infeasibility counter in the solution */
		int infeasibility_counter_;
			
		/** @brief Maximun value of the infeasibility counter */
		int infeasibility_hack_counter_max_;
			
		/** @brief States error weight matrix */
		Eigen::MatrixXd Q_;
			
		/** @brief Terminal states error weight matrix */
		Eigen::MatrixXd P_;
			
		/** @brief Input error weight matrix */
		Eigen::MatrixXd R_;
			
		/** @brief Vector of the MPC solution */
		double *mpc_solution_;
			
		/** @brief Stationary control signal for the reference state vector */
		Eigen::MatrixXd u_reference_;
			
		/** @brief Control signal computes for MPC */
		double *control_signal_;
			
		/** @brief Data of the time vector of the system */
		std::vector<int> t_;
			
		/** @brief Data of the state vector of the system */
		std::vector<std::vector<double> > x_;
			
		/** @brief Data of the reference state vector of the system */
		std::vector<std::vector<double> > xref_;
			
		/** @brief Data of the control signal vector of the system */
		std::vector<std::vector<double> > u_;
			
		/** @brief Path where the data will be save */
		std::string path_name_;
			
		/** @brief Name of the file where the data will be save */
		std::string data_name_;
			
		/** @brief Label that indicates if it will be save the data */
		bool enable_record_;
};

} //@namespace locomotion
} //@namespace dwl


inline double* dwl::locomotion::ModelPredictiveControl::getControlSignal() const
{
	for (int i = 0; i < inputs_; i++) {
		if (infeasibility_counter_ < infeasibility_hack_counter_max_)
			control_signal_[i] = mpc_solution_[infeasibility_counter_ * inputs_ + i];
		else
			control_signal_[i] = u_reference_(i);
	}

	return control_signal_;
}

#endif
