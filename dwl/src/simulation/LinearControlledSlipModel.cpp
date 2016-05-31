#include <dwl/simulation/LinearControlledSlipModel.h>


namespace dwl
{

namespace simulation
{

LinearControlledSlipModel::LinearControlledSlipModel() : init_model_(false),
		init_response_(false), slip_omega_(0.)
{

}


LinearControlledSlipModel::~LinearControlledSlipModel()
{

}


void LinearControlledSlipModel::setModelProperties(SlipProperties model)
{
	slip_ = model;
	init_model_ = true;
}


void LinearControlledSlipModel::initResponse(const ReducedBodyState& state,
											 const SlipControlParams& params)
{
	if (!init_model_) {
		printf(YELLOW "Warning: could not initialized the initResponse because"
				" there is not defined the SLIP model\n" COLOR_RESET);
		return;
	}

	// Saving the initial state
	initial_state_ = state;

	// Saving the SLIP control params
	params_ = params;

	// Computing the coefficients of the Spring Loaded Inverted Pendulum
	// (SLIP) response
	double slip_height = initial_state_.com_pos(rbd::Z) - initial_state_.cop(rbd::Z);
	slip_omega_ = sqrt(slip_.gravity / slip_height);
	double alpha = 2 * slip_omega_ * params_.duration;
	Eigen::Vector2d slip_hor_proj = (initial_state_.com_pos - initial_state_.cop).head<2>();
	Eigen::Vector2d slip_hor_disp = initial_state_.com_vel.head<2>() * params_.duration;
	beta_1_ = slip_hor_proj / 2 +
			(slip_hor_disp - params_.cop_shift.head<2>()) / alpha;
	beta_2_ = slip_hor_proj / 2 -
			(slip_hor_disp - params_.cop_shift.head<2>()) / alpha;

	init_response_ = true;
}


void LinearControlledSlipModel::computeResponse(ReducedBodyState& state,
												double time)
{
	if (!init_response_) {
		printf(YELLOW "Warning: could not be computed the SLIP response because."
				" You should call initResponse\n" COLOR_RESET);
		return;
	}

	// Checking the preview duration
	if (time < initial_state_.time)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time


	// Computing the delta time w.r.t. the initial time
	double dt = time - initial_state_.time;
	state.time = time;

	// Computing the horizontal motion of the CoM according to
	// the SLIP system
	Eigen::Vector2d beta_exp_1 = beta_1_ * exp(slip_omega_ * dt);
	Eigen::Vector2d beta_exp_2 = beta_2_ * exp(-slip_omega_ * dt);
	Eigen::Vector2d cop_T = params_.cop_shift.head<2>() / params_.duration;
	state.com_pos.head<2>() =
			beta_exp_1 + beta_exp_2 +
			cop_T * dt +
			initial_state_.cop.head<2>();
	state.com_vel.head<2>() =
			slip_omega_ * beta_exp_1 -
			slip_omega_ * beta_exp_2 +
			cop_T;
	state.com_acc.head<2>() =
			slip_omega_ * slip_omega_ * beta_exp_1 +
			slip_omega_ * slip_omega_ * beta_exp_2;

	// There is not vertical motion of the CoM
	state.com_pos(rbd::Z) = initial_state_.com_pos(rbd::Z);
	state.com_vel(rbd::Z) = 0.;
	state.com_acc(rbd::Z) = 0.;

	// Computing the CoP position given the linear assumption
	state.cop = initial_state_.cop + (dt / params_.duration) * params_.cop_shift;
}


void LinearControlledSlipModel::computeSystemEnergy(Eigen::Vector3d& com_energy,
													const ReducedBodyState& initial_state,
													const SlipControlParams& params)
{
	// Initialization the coefficient of the LC-SLIP model
	initResponse(initial_state, params);

	// Computing the CoM energy associated to the horizontal
	// dynamics
	// x_acc^2 = (beta1 * slip_omega^2)^2 * exp(2 * slip_omega * dt)
	//			 (beta2 * slip_omega^2)^2 * exp(-2 * slip_omega * dt)
	//			 beta1 * beta2 * slip_omega^4
	double dt = params.duration;
	Eigen::Vector2d c_1 = (beta_1_ * pow(slip_omega_,2)).array().pow(2);
	Eigen::Vector2d c_2 = (beta_2_ * pow(slip_omega_,2)).array().pow(2);
	Eigen::Vector2d c_3 = beta_1_.cwiseProduct(beta_2_);
	com_energy.head<2>() =
			c_1 * exp(2 * slip_omega_ * dt) +
			c_2 * exp(-2 * slip_omega_ * dt) +
			c_3 * pow(slip_omega_,4);

	// There is not energy associated to the vertical movement
	com_energy(rbd::Z) = 0;
}

} //@namespace simulation
} //@namespace dwl
