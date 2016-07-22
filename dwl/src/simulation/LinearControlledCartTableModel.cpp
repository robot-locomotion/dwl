#include <dwl/simulation/LinearControlledCartTableModel.h>


namespace dwl
{

namespace simulation
{

LinearControlledCartTableModel::LinearControlledCartTableModel() :
		init_model_(false),	init_response_(false), height_(0.), omega_(0.)
{

}


LinearControlledCartTableModel::~LinearControlledCartTableModel()
{

}


void LinearControlledCartTableModel::setModelProperties(CartTableProperties model)
{
	properties_ = model;
	init_model_ = true;
}


void LinearControlledCartTableModel::initResponse(const ReducedBodyState& state,
											 	  const CartTableControlParams& params)
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

	// Computing the coefficients of the Cart-Table response
	height_ = initial_state_.com_pos(rbd::Z) - initial_state_.cop(rbd::Z);
	omega_ = sqrt(properties_.gravity / height_);
	double alpha = 2 * omega_ * params_.duration;
	Eigen::Vector2d hor_proj = (initial_state_.com_pos - initial_state_.cop).head<2>();
	Eigen::Vector2d hor_disp = initial_state_.com_vel.head<2>() * params_.duration;
	beta_1_ = hor_proj / 2 +
			(hor_disp - params_.cop_shift.head<2>()) / alpha;
	beta_2_ = hor_proj / 2 -
			(hor_disp - params_.cop_shift.head<2>()) / alpha;
	cop_T_ = params_.cop_shift.head<2>() / params_.duration;


	// Getting the support vertices
	std::vector<Eigen::Vector3f> vertices;
	vertices.resize(initial_state_.support_region.size());
	unsigned int v_idx = 0;
	for (rbd::BodyPosition::const_iterator v = initial_state_.support_region.begin();
			v != initial_state_.support_region.end(); v++) {
		vertices[v_idx] = v->second.cast<float>();
		++v_idx;
	}

	// Computing the normal vector of the support region
	math::computePlaneParameters(support_normal_, vertices);

	init_response_ = true;
}


void LinearControlledCartTableModel::computeResponse(ReducedBodyState& state,
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

	// Computing the CoP position given the linear assumption
	Eigen::Vector3d delta_cop = (dt / params_.duration) * params_.cop_shift;
	state.cop = initial_state_.cop + delta_cop;

	// Computing the horizontal motion of the CoM according to
	// the cart-table system
	Eigen::Vector2d beta_exp_1 = beta_1_ * exp(omega_ * dt);
	Eigen::Vector2d beta_exp_2 = beta_2_ * exp(-omega_ * dt);
	state.com_pos.head<2>() =
			beta_exp_1 + beta_exp_2 +
			cop_T_ * dt +
			initial_state_.cop.head<2>();
	state.com_vel.head<2>() =
			omega_ * beta_exp_1 -
			omega_ * beta_exp_2 +
			cop_T_;
	state.com_acc.head<2>() =
			omega_ * omega_ * beta_exp_1 +
			omega_ * omega_ * beta_exp_2;


	// Computing the Z-component of the CoP
	// From the plane equation (i.e.  n * p = 0), we derive the following
	// equation that allows us to compute the delta in z
	Eigen::Vector2d normal_2d = support_normal_.head<2>();
	double normal_z = support_normal_(rbd::Z);
	double delta_posz = -(normal_2d.dot(delta_cop.head<2>())) / normal_z;

	Eigen::Vector2d cop_vel = Eigen::Vector2d::Ones() / params_.duration;

	// There is not vertical motion of the CoM. Note that we derive the above
	// mentioned equation in order to get the velocity and acceleration components
	state.com_pos(rbd::Z) = initial_state_.com_pos(rbd::Z) + delta_posz;
	state.com_vel(rbd::Z) = -(normal_2d.dot(cop_vel)) / normal_z;
	state.com_acc(rbd::Z) = 0.;
	state.cop(rbd::Z) = initial_state_.cop(rbd::Z) + delta_posz;
}


void LinearControlledCartTableModel::computeSystemEnergy(Eigen::Vector3d& com_energy,
														 const ReducedBodyState& initial_state,
														 const CartTableControlParams& params)
{
	// Initialization the coefficient of the cart-table model
	initResponse(initial_state, params);

	// Computing the CoM energy associated to the horizontal
	// dynamics
	// x_acc^2 = (beta1 * omega^2)^2 * exp(2 * omega * dt)
	//			 (beta2 * omega^2)^2 * exp(-2 * omega * dt)
	//			 beta1 * beta2 * slip_omega^4
	double dt = params.duration;
	c_1_ = (beta_1_ * pow(omega_,2)).array().pow(2);
	c_2_ = (beta_2_ * pow(omega_,2)).array().pow(2);
	c_3_ = beta_1_.cwiseProduct(beta_2_) * pow(omega_,4);
	com_energy.head<2>() =
			c_1_ * exp(2 * omega_ * dt) +
			c_2_ * exp(-2 * omega_ * dt) +
			c_3_;

	// There is not energy associated to the vertical movement
	com_energy(rbd::Z) = 0;
}


double LinearControlledCartTableModel::getPendulumHeight()
{
	return height_;
}

} //@namespace simulation
} //@namespace dwl
