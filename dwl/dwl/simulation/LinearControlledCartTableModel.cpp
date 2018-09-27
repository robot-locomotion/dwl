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
											 	  const CartTableControlParams& params_H)
{
	if (!init_model_) {
		printf(YELLOW "Warning: could not initialized the initResponse because"
				" there is not defined the cart-table model\n" COLOR_RESET);
		return;
	}

	// Initializes the CoM response
	initCoMResponse(state, params_H);

	// Initializes the attitude response
	initAttitudeResponse(state, params_H);

	init_response_ = true;
}


void LinearControlledCartTableModel::initCoMResponse(const ReducedBodyState& state,
													 const CartTableControlParams& params_H)
{
	// Saving the initial state
	initial_state_ = state;

	// Saving the cart-table control params in the world frame
	params_W_.duration = params_H.duration;
	params_W_.cop_shift =
			frame_tf_.fromHorizontalToWorldFrame(params_H.cop_shift,
												 initial_state_.getCoMSE3().getRPY());

	// Computing the coefficients of the cart-table response
	height_ = initial_state_.getCoMSE3().getTranslation()(rbd::Z) -
			  initial_state_.getCoPPosition_W()(rbd::Z);
	omega_ = sqrt(properties_.gravity / height_);
	double alpha = 2 * omega_ * params_W_.duration;
	Eigen::Vector2d hor_proj =
			(initial_state_.getCoMSE3().getTranslation() -
					initial_state_.getCoPPosition_W()).head<2>();
	Eigen::Vector2d hor_disp =
			initial_state_.getCoMVelocity_W().getLinear().head<2>() *
			params_W_.duration;
	beta_1_ = hor_proj / 2 +
			(hor_disp - params_W_.cop_shift.head<2>()) / alpha;
	beta_2_ = hor_proj / 2 -
			(hor_disp - params_W_.cop_shift.head<2>()) / alpha;
	cop_T_ = params_W_.cop_shift.head<2>() / params_W_.duration;
}


void LinearControlledCartTableModel::initAttitudeResponse(const ReducedBodyState& state,
		 	 	 	 	 	 	 	 	 	 	 	 	  const CartTableControlParams& params_H)
{
	// Saving the initial state
	initial_state_ = state;

	// Getting the support vertices
	std::vector<Eigen::Vector3f> vertices;
	vertices.resize(initial_state_.support_region.size());
	unsigned int v_idx = 0;
	for (SE3Map::const_iterator v = initial_state_.support_region.begin();
			v != initial_state_.support_region.end(); ++v) {
		vertices[v_idx] = v->second.getTranslation().cast<float>();
		++v_idx;
	}

	// Computing the normal vector of the support region
	math::computePlaneParameters(support_normal_, vertices);

	// Getting the RPY angle of the next support region
	Eigen::Vector3d ref_dir = Eigen::Vector3d::UnitZ();
	Eigen::Quaterniond support_orientation;
	support_orientation.setFromTwoVectors(ref_dir, support_normal_);
	support_rpy_ = math::getRPY(support_orientation);

	// Computing the require roll and pitch velocities
	double roll_delta = support_rpy_(0) - initial_state_.getCoMSE3().getRPY()(0);
	double pitch_delta = support_rpy_(1) - initial_state_.getCoMSE3().getRPY()(1);
	double roll_vel = roll_delta / params_W_.duration;
	double pitch_vel = pitch_delta / params_W_.duration;
	Eigen::Vector3d initial_rpyd = initial_state_.getRPYVelocity_W();
	Eigen::Vector3d initial_rpydd = initial_state_.getRPYAcceleration_W();

	// Saturating the maximum roll and pitch displacement in case that we
	// reach maximum velocity
	double max_roll_vel = 0.1;
	double max_pitch_vel = 0.1;
	math::Spline::Point start_roll(initial_state_.getCoMSE3().getRPY()(0),
								   initial_rpyd(0),
								   initial_rpydd(0));
	math::Spline::Point end_roll;
	if (fabs(roll_vel) > max_roll_vel) {
		double final_roll;
		if (roll_vel >= 0.) {
			final_roll =
					initial_state_.getCoMSE3().getRPY()(0) +
					max_roll_vel * params_W_.duration;
			end_roll = math::Spline::Point(final_roll,
										   max_roll_vel,
										   max_roll_vel / params_W_.duration);
		} else {
			final_roll =
					initial_state_.getCoMSE3().getRPY()(0) -
					max_roll_vel * params_W_.duration;
			end_roll = math::Spline::Point(final_roll,
										   -max_roll_vel,
										   -max_roll_vel / params_W_.duration);
		}
	} else {
		end_roll = math::Spline::Point(support_rpy_(0),
									   roll_vel,
									   roll_vel / params_W_.duration);
	}

	math::Spline::Point start_pitch(initial_state_.getCoMSE3().getRPY()(1),
									initial_rpyd(1),
									initial_rpydd(1));
	math::Spline::Point end_pitch;
	if (fabs(pitch_vel) > max_pitch_vel) {
		double final_pitch;
		if (pitch_vel >= 0.) {
			final_pitch =
					initial_state_.getCoMSE3().getRPY()(1) +
					max_pitch_vel * params_W_.duration;
			end_pitch = math::Spline::Point(final_pitch,
											max_pitch_vel,
											max_pitch_vel / params_W_.duration);
		} else {
			final_pitch =
					initial_state_.getCoMSE3().getRPY()(1) -
					max_pitch_vel * params_W_.duration;
			end_pitch = math::Spline::Point(final_pitch,
											-max_pitch_vel,
											-max_pitch_vel / params_W_.duration);
		}
	} else {
		end_pitch = math::Spline::Point(support_rpy_(1),
										pitch_vel,
										pitch_vel / params_W_.duration);
	}

	// Initializing the splines
	roll_spline_.setBoundary(0., params_W_.duration,
							 start_roll,
							 end_roll);
	pitch_spline_.setBoundary(0., params_W_.duration,
							 start_pitch,
							 end_pitch);
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

	// Computing the CoM response
	computeCoMResponse(state, time);

	// Computing the attitude response
	computeAttitudeResponse(state, time);
}


void LinearControlledCartTableModel::computeCoMResponse(ReducedBodyState& state,
				 	 	 	 							double time)
{
	// Computing the delta time w.r.t. the initial time
	double dt = time - initial_state_.time;
	state.setTime(time);

	// Computing the CoP position given the linear assumption
	Eigen::Vector3d delta_cop = (dt / params_W_.duration) * params_W_.cop_shift;

	// Computing the Z-component of the CoP
	// From the plane equation (i.e. n * p = 0), we derive the following
	// equation that allows us to compute the delta in z
	Eigen::Vector2d normal_2d = support_normal_.head<2>();
	double normal_z = support_normal_(rbd::Z);
	double delta_posz = -(normal_2d.dot(delta_cop.head<2>())) / normal_z;

	// Updating the CoP
	state.setCoPPosition_W(
			initial_state_.getCoPPosition_W() + delta_cop +
			Eigen::Vector3d(0., 0., delta_posz));

	// Computing the horizontal motion of the CoM according to
	// the cart-table system
	Eigen::Vector2d beta_exp_1 = beta_1_ * exp(omega_ * dt);
	Eigen::Vector2d beta_exp_2 = beta_2_ * exp(-omega_ * dt);
	state.com_pos.data.translation().head<2>() =
			beta_exp_1 + beta_exp_2 +
			cop_T_ * dt +
			initial_state_.cop.head<2>();
	state.com_vel.data.linear().head<2>() =
			omega_ * beta_exp_1 -
			omega_ * beta_exp_2 +
			cop_T_;
	state.com_acc.data.linear().head<2>() =
			omega_ * omega_ * beta_exp_1 +
			omega_ * omega_ * beta_exp_2;

	// There is not vertical motion of the CoM. Note that we derive the above
	// mentioned equation in order to get the velocity and acceleration components
	// Note that cop_T_ is equal to the CoP velocity in the horizontal frame
	state.com_pos.data.translation()(rbd::Z) =
			initial_state_.com_pos.data.translation()(rbd::Z) + delta_posz;
	state.com_vel.data.linear()(rbd::Z) = -(normal_2d.dot(cop_T_)) / normal_z;
	state.com_acc.data.linear()(rbd::Z) = 0.;

	// Updating the support region
	state.setSupportRegion(initial_state_.getSupportRegion());
}


void LinearControlledCartTableModel::computeAttitudeResponse(ReducedBodyState& state,
							 	 	 	 	 	 	 	 	 double time)
{
	// Splinning the roll and pitch angle in order to have the base frame
	// parallel with the support frame
	double dt = time - initial_state_.time;
	math::Spline::Point roll, pitch, yaw;
	roll_spline_.getPoint(dt, roll);
	pitch_spline_.getPoint(dt, pitch);
	yaw.x = initial_state_.getCoMSE3().getRPY()(2);
	yaw.xd = initial_state_.getCoMVelocity_W().getAngular()(2);
	yaw.xdd = initial_state_.getCoMAcceleration_W().getAngular()(2);
	Eigen::Vector3d rpy(roll.x, pitch.x, yaw.x);
	Eigen::Vector3d rpy_vel(roll.xd, pitch.xd, yaw.xd);
	Eigen::Vector3d rpy_acc(roll.xdd, pitch.xdd, yaw.xdd);
	state.setCoMSE3(dwl::SE3(state.getCoMSE3().getTranslation(), rpy));
	state.setRPYVelocity_W(rpy_vel);
	state.setRPYAcceleration_W(rpy_acc);
}


double LinearControlledCartTableModel::getPendulumHeight()
{
	return height_;
}

} //@namespace simulation
} //@namespace dwl
