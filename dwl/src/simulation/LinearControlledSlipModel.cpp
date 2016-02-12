#include <dwl/simulation/LinearControlledSlipModel.h>


namespace dwl
{

namespace simulation
{

LinearControlledSlipModel::LinearControlledSlipModel() : init_model_(false),
		init_response_(false), initial_length_(0.), slip_omega_(0.),
		spring_omega_(0.), d_1_(0.), d_2_(0.)
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

	// Computing the initial length of the pendulum
	initial_length_ = (initial_state_.com_pos - initial_state_.cop).norm();

	// Computing the coefficients of the spring-mass system response
	spring_omega_ = sqrt(slip_.stiffness / slip_.mass);
	d_1_ = initial_state_.com_pos(rbd::Z) - initial_length_ +
			slip_.gravity /	pow(spring_omega_,2);
	d_2_ = initial_state_.com_vel(rbd::Z) / spring_omega_ -
			params_.length_shift / (spring_omega_ * params_.duration);

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
	state.com_pos.head<2>() =
			beta_1_ * exp(slip_omega_ * dt) +
			beta_2_ * exp(-slip_omega_ * dt) +
			(params_.cop_shift.head<2>() / params_.duration) * dt +
			initial_state_.cop.head<2>();
	state.com_vel.head<2>() =
			beta_1_ * slip_omega_ * exp(slip_omega_ * dt) -
			beta_2_ * slip_omega_ * exp(-slip_omega_ * dt) +
			params_.cop_shift.head<2>() / params_.duration;
	state.com_acc.head<2>() =
			beta_1_ * pow(slip_omega_,2) * exp(slip_omega_ * dt) +
			beta_2_ * pow(slip_omega_,2) * exp(-slip_omega_ * dt);

	// Computing the vertical motion of the CoM according to
	// the spring-mass system
	state.com_pos(rbd::Z) =
			d_1_ * cos(spring_omega_ * dt) +
			d_2_ * sin(spring_omega_ * dt) +
			(params_.length_shift / params_.duration) * dt +
			initial_length_ - slip_.gravity / pow(spring_omega_,2);
	state.com_vel(rbd::Z) =
			-d_1_ * spring_omega_ * sin(spring_omega_ * dt) +
			d_2_ * spring_omega_ * cos(spring_omega_ * dt) +
			params_.length_shift / params_.duration;
	state.com_acc(rbd::Z) =
			-d_1_ * pow(spring_omega_,2) * cos(spring_omega_ * dt) -
			d_2_ * pow(spring_omega_,2) * sin(spring_omega_ * dt);
	state.com_pos(rbd::Z) = initial_state_.com_pos(rbd::Z);//TODO for debugging
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

	// Computing the CoM energy associated to the vertical
	// dynamics
	// z_acc^2 = (d1 * spring_omega^2)^4 * cos^2(spring_omega_ * dt)) +
	//			 (d2 * spring_omega^2)^4 * sin^2(spring_omega_ * dt)) +
	//			 (d1 * d2 * spring_omega^4) * cos(spring_omega_ * dt) * sin(spring_omega_ * dt)
	com_energy(rbd::Z) =
			pow(d_1_,2) * pow(spring_omega_,4) *
			(0.5 * dt + 0.25 * sin(spring_omega_ * dt) * cos(spring_omega_ * dt)) +
			pow(d_2_,2) * pow(spring_omega_,4) * (0.5 * dt + 0.25 * sin(2 * spring_omega_ * dt)) +
			d_1_ * d_2_ * pow(spring_omega_,4) * 0.5 * pow(sin(spring_omega_ * dt),2);
	com_energy(rbd::Z) = 0; // TODO for debugging
}

} //@namespace simulation
} //@namespace dwl
