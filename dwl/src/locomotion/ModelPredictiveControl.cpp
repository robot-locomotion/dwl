#include <dwl/locomotion/ModelPredictiveControl.h>



namespace dwl
{

namespace locomotion
{

ModelPredictiveControl::ModelPredictiveControl() : model_(NULL), optimizer_(NULL)
{
	enable_record_ = true;
}


ModelPredictiveControl::~ModelPredictiveControl()
{

}


bool ModelPredictiveControl::reset(model::LinearDynamicalSystem* model,
								   solver::QuadraticProgram* optimizer)
{
	// Setting of the pointer of the model and optimizer classes
	model_ = model;
	optimizer_ = optimizer;

	time_index_ = 0;
	
	// Reading of the horizon value of the model predictive control algorithm
	nh_.param<int>("horizon", horizon_, 30);
	ROS_INFO("Got param: horizon = %d", horizon_);
	
	nh_.param<int>("infeasibility_hack_counter_max", infeasibility_hack_counter_max_, 1);
	ROS_INFO("Got param: infeasibility_hack_counter_max = %d", infeasibility_hack_counter_max_);

	
	// Reading of the problem variables
	states_ = model_->getStatesNumber();
	inputs_ = model_->getInputsNumber();
	outputs_ = model_->getOutputsNumber();
	
	variables_ = horizon_ * inputs_;
	constraints_ = optimizer_->getConstraintNumber();


	// Initializing the QP solver
	if (!optimizer_->init(inputs_ * horizon_, constraints_ * horizon_))
		return false;

	printf("Reset successful. States = %d \n Inputs = %d \n Outputs = %d \n Constraints = %d \n",
			states_, inputs_, outputs_, constraints_);
	return true;
}



bool ModelPredictiveControl::init()
{
	// Initialization of MPC solution
	operation_states_ = new double[states_];
	operation_inputs_ = new double[inputs_];
	
	
	u_reference_ = Eigen::MatrixXd::Zero(inputs_, 1);
	infeasibility_counter_ = 0;
	x_.resize(states_);
	xref_.resize(states_);
	u_.resize(inputs_);
	
	
	// Initialization of state space matrices
	A_ = Eigen::MatrixXd::Zero(states_, states_);
	B_ = Eigen::MatrixXd::Zero(states_, inputs_);
	C_ = Eigen::MatrixXd::Zero(outputs_, states_);
	
	// Obtention of the model parameters
	if (model_->computeLinearSystem(A_, B_)) {
		ROS_INFO("Model calculated successfully.");
		std::cout << "A\n" << A_ << std::endl;
		std::cout << "B\n" << B_ << std::endl;
	}
	
	// Reading the weight matrices of the cost function
	Q_ = Eigen::MatrixXd::Zero(states_, states_);
	P_ = Eigen::MatrixXd::Zero(states_, states_);
	R_ = Eigen::MatrixXd::Zero(inputs_, inputs_);
	XmlRpc::XmlRpcValue Q_list, P_list, R_list;
	nh_.getParam("optimizer/states_error_weight_matrix/data", Q_list);
	ROS_ASSERT(Q_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(Q_list.size() == states_ * states_);

	nh_.getParam("optimizer/terminal_state_weight_matrix/data", P_list);
	ROS_ASSERT(P_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(P_list.size() == states_ * states_);
	
	int z = 0;
	for (int i = 0; i < states_; i++) {
		for (int j = 0; j < states_; j++) {
			ROS_ASSERT(Q_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			Q_(i, j) = static_cast<double>(Q_list[z]);
			
			ROS_ASSERT(P_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			P_(i, j) = static_cast<double>(P_list[z]);
			z++;
		}
	}
	
	nh_.getParam("optimizer/input_error_weight_matrix/data", R_list);
	ROS_ASSERT(R_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(R_list.size() == inputs_ * inputs_);
	z = 0;
	for (int i = 0; i < inputs_; i++) {
		for (int j = 0; j < inputs_; j++) {
			ROS_ASSERT(R_list[z].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			R_(i, j) = static_cast<double>(R_list[z]);
			z++;
		}
	}
	
	
	// Creation of the states and inputs weight matrices for the quadratic program
	Q_bar_ = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, (horizon_ + 1) * states_);
	R_bar_ = Eigen::MatrixXd::Zero(horizon_ * inputs_, horizon_ * inputs_);
	for (int i = 0; i < horizon_; i++) {
		Q_bar_.block(i * states_, i * states_, states_, states_) = Q_;
		R_bar_.block(i * inputs_, i * inputs_, inputs_, inputs_) = R_;
	}
	Q_bar_.block(horizon_ * states_, horizon_ * states_, states_, states_) = P_;
	
	
	// Reading the constraint vectors
	Eigen::VectorXd lbG = Eigen::VectorXd::Zero(constraints_);
	Eigen::VectorXd ubG = Eigen::VectorXd::Zero(constraints_);
	XmlRpc::XmlRpcValue lbG_list, ubG_list;
	nh_.getParam("optimizer/constraints/constraint_vector_low", lbG_list);
	ROS_ASSERT(lbG_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	nh_.getParam("optimizer/constraints/constraint_vector_upp", ubG_list);
	ROS_ASSERT(ubG_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	if (ubG_list.size() == lbG_list.size()) {
		for (int i = 0; i < ubG_list.size(); ++i) {
			ROS_ASSERT(lbG_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			lbG(i) = static_cast<double>(lbG_list[i]);
			
			ROS_ASSERT(ubG_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			ubG(i) = static_cast<double>(ubG_list[i]);
		}
	
	}
	
	// Reading the bound vectors
	Eigen::VectorXd lb = Eigen::VectorXd::Zero(inputs_);
	Eigen::VectorXd ub = Eigen::VectorXd::Zero(inputs_);
	XmlRpc::XmlRpcValue lb_list, ub_list;
	nh_.getParam("optimizer/constraints/bound_vector_low", lb_list);
	ROS_ASSERT(lb_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	nh_.getParam("optimizer/constraints/bound_vector_upp", ub_list);
	ROS_ASSERT(ub_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	if (ub_list.size() == lb_list.size()) {
		for (int i = 0; i < ub_list.size(); ++i) {
			ROS_ASSERT(lb_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			lb(i) = static_cast<double>(lb_list[i]);                              
			
			ROS_ASSERT(ub_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			ub(i) = static_cast<double>(ub_list[i]);
		}
	}
	
	// Reading the state bound matrix
	Eigen::MatrixXd M = Eigen::MatrixXd::Zero(constraints_, states_);
	XmlRpc::XmlRpcValue M_list;
	nh_.getParam("optimizer/constraints/constraint_matrix_M", M_list);
	ROS_ASSERT(M_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(M_list.size() == constraints_ * states_);
	
	z = 0;
	for (int i = 0; i < constraints_; ++i) {
		for (int j = 0; j < states_; j++) {
			ROS_ASSERT(M_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			M(i, j) = static_cast<double>(M_list[z]);
			z++;
		}
	}
	
	
	// Creation of the extended constraint and bound vector
	lbG_bar_ = Eigen::VectorXd::Zero(constraints_ * horizon_);
	ubG_bar_ = Eigen::VectorXd::Zero(constraints_ * horizon_);
	lb_bar_ = Eigen::VectorXd::Zero(inputs_ * horizon_);
	ub_bar_ = Eigen::VectorXd::Zero(inputs_ * horizon_);
	for (int i = 0; i < horizon_; i++) {
		lbG_bar_.block(i * constraints_, 0, constraints_, 1) = lbG;
		ubG_bar_.block(i * constraints_, 0, constraints_, 1) = ubG;
		lb_bar_.block(i * inputs_, 0, inputs_, 1) = lb;
		ub_bar_.block(i * inputs_, 0, inputs_, 1) = ub;
	}
	
	// Creation of the extended constraint matrix G_bar_
	M_bar_ = Eigen::MatrixXd::Zero(constraints_ * horizon_, (horizon_ + 1) * states_);
	for (int i = 0; i < horizon_; i++) {
		for (int j = 0; j < horizon_ + 1; j++) {
			if (i == j) {
				M_bar_.block(i * constraints_, j * states_, constraints_, states_) = M;
			}
		}
	}
	
	
	printf("STDMPC class successfully initialized.");
	return true;
}



void ModelPredictiveControl::update(WholeBodyState& current_state,
									WholeBodyState& reference_state)
{
	Eigen::Map<Eigen::VectorXd> x_measured_eigen(x_measured, states_, 1);// current_state
	Eigen::Map<Eigen::VectorXd> x_reference_eigen(x_reference, states_, 1);// reference state

	// Update of the model parameters
	Eigen::MatrixXd A, B;
	if (model_->getModelType()) {
		model_->computeLinearSystem(A, B);
	}
	
	// Compute steady state control based on updated system matrices
	Eigen::JacobiSVD<Eigen::MatrixXd> SVD_B(B, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u_reference_ = SVD_B.solve(x_reference_eigen - A * x_reference_eigen);
	
	// Creation of the base vector
	std::vector<Eigen::MatrixXd> A_pow;
	A_pow.push_back(Eigen::MatrixXd::Identity(states_, states_));
	for (int i = 1; i < horizon_ + 1; i++) {
		Eigen::MatrixXd A_pow_i = A_pow[i-1] * A;
		A_pow.push_back(A_pow_i);
	}

	// Computing the extended state and input matrixes for the predefined horizon
	Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, states_);
	Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, horizon_ * inputs_);
	Eigen::MatrixXd x_ref_bar = Eigen::MatrixXd::Zero((horizon_ + 1) * states_, 1);
	for (int i = 0; i < horizon_ + 1; i++) {
		for (int j = 0; j < horizon_; j++) {
			if (i == horizon_) {
				if (j == 0) {
					A_bar.block(i * states_, 0, states_, states_) = A_pow[i];
					x_ref_bar.block(i * states_, 0, states_, 1) = x_reference_eigen;
				}
				if (i > j)
					B_bar.block(i * states_, j * inputs_, states_, inputs_) = A_pow[i-j-1] * B;
			} else {
				if (j == 0) {
					A_bar.block(i * states_, 0, states_, states_) = A_pow[i];
					x_ref_bar.block(i * states_, 0, states_, 1) = x_reference_eigen;
				}
				if (i > j)
					B_bar.block(i * states_, j * inputs_, states_, inputs_) = A_pow[i-j-1] * B;
			}
		}
	}
	
	// Computing the values of the Hessian matrix and Gradient vector
	Eigen::MatrixXd hessian = Eigen::Matrix::Zero(horizon_ * inputs_, horizon_ * inputs_);
	Eigen::VectorXd gradient = Eigen::VectorXd::Zero(horizon_ * inputs_);
	hessian = B_bar.transpose() * Q_bar_ * B_bar + R_bar_;
	gradient = B_bar.transpose() * Q_bar_ * (A_bar * x_measured_eigen - x_ref_bar);
	
	// Computing the constraint and state bounds for the predefined horizon;
	Eigen::VectorXd lbG_bar = Eigen::VectorXd::Zero(horizon_ * constraints_);
	Eigen::VectorXd ubG_bar = Eigen::VectorXd::Zero(horizon_ * constraints_);
	Eigen::VectorXd lb_bar = Eigen::VectorXd::Zero(horizon_ * inputs_);
	Eigen::VectorXd ub_bar = Eigen::VectorXd::Zero(horizon_ * inputs_);
	model_->getConstraintsBounds(lbG_bar, ubG_bar);
	model_->getStateBounds(lb_bar, ub_bar);
	lbG_bar = lbG_bar - M_bar_ * A_bar * x_measured_eigen;
	ubG_bar = ubG_bar - M_bar_ * A_bar * x_measured_eigen;

	// Computing the extended constraint matrix
	Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(horizon_ * constraints_,
															  horizon_ * inputs_);
	constraint_matrix = M_bar_ * B_bar;
	
	// Solving the QP problem
	double cputime = 0.008;//1.0;//NULL;
	bool success = false;
	success = optimizer_->compute(hessian, gradient,
								  constraint_matrix,
								  lb_bar, ub_bar,
								  lbG_bar, ubG_bar,
								  cputime);
	if (success) {
		mpc_solution_ = optimizer_->getOptimalSolution();
		infeasibility_counter_ = 0;
	} else {
		infeasibility_counter_++;
		printf("Warning: an optimal solution could not be obtained.");
	}

	// Save the data of the MPC
	for (int i = 0; i < states_; i++) {
		x_[i].push_back(x_measured[i]);
		xref_[i].push_back(x_reference[i]);
	}
	t_.push_back(time_index_);
	time_index_++;

	double *u = getControlSignal();
	for (int i = 0; i < inputs_; i++) {
		u_[i].push_back(u[i]);
	}
}

} //@namespace locomotion
} //@namespace dwl
