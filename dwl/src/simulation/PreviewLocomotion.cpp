#include <dwl/simulation/PreviewLocomotion.h>


namespace dwl
{

namespace simulation
{

PreviewLocomotion::PreviewLocomotion() : sample_time_(0.001), gravity_(9.81),
		mass_(0.), step_height_(0.1), force_threshold_(0.)
{
	actual_system_com_.setZero();
}


PreviewLocomotion::~PreviewLocomotion()
{

}


void PreviewLocomotion::resetFromURDFFile(std::string urdf_file,
										  std::string system_file)
{
	resetFromURDFFile(urdf_model::fileToXml(urdf_file), system_file);
}


void PreviewLocomotion::resetFromURDFModel(std::string urdf_model,
										   std::string system_file)
{
	// Resetting the model of the floating-base system
	system_.resetFromURDFModel(urdf_model, system_file);

	// Initializing the dynamics from the URDF model
	dynamics_.modelFromURDFModel(urdf_model, system_file);

	// Setting the gravity magnitude from the rigid-body dynamic model
	gravity_ = system_.getRBDModel().gravity.norm();

	// Getting the total mass of the system
	mass_ = system_.getTotalMass();

	// Getting the floating-base CoM
	actual_system_com_ = system_.getFloatingBaseCoM();
}


void PreviewLocomotion::setSampleTime(double sample_time)
{
	sample_time_ = sample_time;
}


void PreviewLocomotion::setModel(const SLIPModel& model)
{
	slip_ = model;
}


void PreviewLocomotion::setStepHeight(double step_height)
{
	step_height_ = step_height;
}


void PreviewLocomotion::setForceThreshold(double force_threshold)
{
	force_threshold_ = force_threshold;
}


void PreviewLocomotion::multiPhasePreview(PreviewTrajectory& trajectory,
										  const PreviewState& state,
										  const PreviewControl& control)
{
	// Clearing the trajectory
	trajectory.clear();

	// Computing the preview for multi-phase
	for (unsigned int k = 0; k < control.params.size(); k++) {
		PreviewTrajectory phase_traj;

		// Getting the preview params of the actual phase
		PreviewParams preview_params = control.params[k];

		// Getting the actual preview state for this phase
		PreviewState actual_state;
		if (k == 0)
			actual_state = state;
		else
			actual_state = trajectory.back();

		// Computing the preview of the actual phase
		if (preview_params.phase.type == STANCE) {
			stancePreview(phase_traj, actual_state, preview_params);

			// Getting the swing shift per foot
			rbd::BodyVector swing_shift;
			for (unsigned int j = 0; j < preview_params.phase.feet.size(); j++) {
				std::string foot_name = preview_params.phase.feet[j];
				Eigen::Vector2d foot_shift_2d =
						(Eigen::Vector2d) control.feet_shift.find(foot_name)->second;

				// Computing the z displacement of the foot from the height map. TODO hard coded
//				Eigen::Vector3d terminal_base_pos = phase_traj.end()->com_pos - actual_system_com_;
//				Eigen::Vector2d foothold_2d = foot_shift_2d + terminal_base_pos.head<2>();
				double z_shift = 0.;

				Eigen::Vector3d foot_shift(foot_shift_2d(dwl::rbd::X),
										   foot_shift_2d(dwl::rbd::Y),
										   z_shift);
				swing_shift[foot_name] = foot_shift;
			}

			// Adding the swing pattern
			SwingParams swing_params(preview_params.duration, swing_shift);
			addSwingPattern(phase_traj, actual_state, swing_params);
		} else {
			flightPreview(phase_traj, actual_state, preview_params);

			// Adding the swing pattern
			SwingParams swing_params(preview_params.duration, rbd::BodyVector()); // no foothold targets
			addSwingPattern(phase_traj, actual_state, swing_params);
		}

		// Appending the actual phase trajectory
		trajectory.insert(trajectory.end(), phase_traj.begin(), phase_traj.end());

		// Sanity action: defining the actual state if there isn't a trajectory
		if (trajectory.size() == 0)
			trajectory.push_back(state);
	}
}


void PreviewLocomotion::stancePreview(PreviewTrajectory& trajectory,
									  const PreviewState& state,
									  const PreviewParams& params)
{
	// Checking the preview duration
	if (params.duration < sample_time_)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time

	// Computing the coefficients of the Spring Loaded Inverted Pendulum
	// (SLIP) response
	double slip_height = state.com_pos(rbd::Z) - state.cop(rbd::Z);
	double slip_omega = sqrt(gravity_ / slip_height);
	double alpha = 2 * slip_omega * params.duration;
	Eigen::Vector2d slip_hor_proj = (state.com_pos - state.cop).head<2>();
	Eigen::Vector2d slip_hor_disp = state.com_vel.head<2>() * params.duration;
	Eigen::Vector2d beta_1 = slip_hor_proj / 2 +
			(slip_hor_disp - params.cop_shift) / alpha;
	Eigen::Vector2d beta_2 = slip_hor_proj / 2 -
			(slip_hor_disp - params.cop_shift) / alpha;

	// Computing the initial length of the pendulum
	double initial_length = (state.com_pos - state.cop).norm();

	// Computing the coefficients of the spring-mass system response
	double spring_omega = sqrt(slip_.stiffness / mass_);
	double d_1 = state.com_pos(rbd::Z) - initial_length + gravity_ /
			pow(spring_omega,2);
	double d_2 = state.com_vel(rbd::Z) / spring_omega -
			params.length_shift / (spring_omega * params.duration);






	// Computing the support region. Note that the support region
	// remains constant during this phase
	PreviewState current_state;
	for (rbd::BodyVector::const_iterator foot_it = state.foot_pos.begin();
			foot_it != state.foot_pos.end(); foot_it++) {
		std::string foot_name = foot_it->first;

		// Adding the foot as support polygon if it's not a swing foot
		if (params.phase.isSwingFoot(foot_name)) {// it's not a swing phase
			Eigen::Vector3d foot_pos_com = foot_it->second;
			Eigen::Vector3d foot_pos_world = foot_pos_com + state.com_pos;
			std::cout << "						foot_pos_world = " << foot_pos_world.transpose() << std::endl;
			current_state.support_region[foot_name] = foot_pos_world;
		}
	}
	std::cout << "###############################" << std::endl;








	// Computing the preview trajectory
	unsigned int num_samples = ceil(params.duration / sample_time_);
	trajectory.resize(num_samples);
	for (unsigned int k = 0; k < num_samples; k++) {
		double time = sample_time_ * (k + 1);

		// Computing the current time of the preview trajectory
		current_state.time = state.time + time;

		// Computing the horizontal motion of the CoM according to
		// the SLIP system
		current_state.com_pos.head<2>() =
				beta_1 * exp(slip_omega * time) +
				beta_2 * exp(-slip_omega * time) +
				(params.cop_shift / params.duration) * time +
				state.cop.head<2>();
		current_state.com_vel.head<2>() =
				beta_1 * slip_omega * exp(slip_omega * time) -
				beta_2 * slip_omega * exp(-slip_omega * time) +
				params.cop_shift / params.duration;
		current_state.com_vel.head<2>() =
				beta_1 * pow(slip_omega,2) * exp(slip_omega * time) +
				beta_2 * pow(slip_omega,2) * exp(-slip_omega * time);

		// Computing the vertical motion of the CoM according to
		// the spring-mass system
		current_state.com_pos(rbd::Z) =
				d_1 * cos(spring_omega * time) +
				d_2 * sin(spring_omega * time) +
				(params.length_shift / params.duration) * time +
				initial_length - gravity_ / pow(spring_omega,2);
		current_state.com_vel(rbd::Z) =
				-d_1 * spring_omega * sin(spring_omega * time) +
				d_2 * spring_omega * cos(spring_omega * time) +
				params.length_shift / params.duration;
		current_state.com_acc(rbd::Z) =
				-d_1 * pow(spring_omega,2) * cos(spring_omega * time) -
				d_2 * pow(spring_omega,2) * sin(spring_omega * time);

		// Computing the CoP position given the linear assumption
		Eigen::Vector3d cop_shift_3d(params.cop_shift(rbd::X),
									 params.cop_shift(rbd::Y),
									 0.);
		current_state.cop = state.cop +	(time / params.duration) * cop_shift_3d;

		// Computing the heading motion according to heading kinematic equation
		current_state.head_pos = state.head_pos + state.head_vel * time +
				0.5 * params.head_acc * pow(time,2);
		current_state.head_vel = state.head_vel + params.head_acc * time;
		current_state.head_acc = params.head_acc;

		// Appending the current state to the preview trajectory
		trajectory[k] = current_state;
	}
}


void PreviewLocomotion::flightPreview(PreviewTrajectory& trajectory,
						   	   	   	  const PreviewState& state,
									  const PreviewParams& params)
{
	// Checking the preview duration
	if (params.duration < sample_time_)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time

	// Setting the gravity vector
	Eigen::Vector3d gravity_vec = Eigen::Vector3d::Zero();
	gravity_vec(rbd::Z) = -gravity_;

	// Computing the preview trajectory
	unsigned int num_samples = ceil(params.duration / sample_time_);
	trajectory.resize(num_samples);
	for (unsigned int k = 0; k < num_samples; k++) {
		double time = sample_time_ * (k + 1);

		// Computing the current time of the preview trajectory
		PreviewState current_state;
		current_state.time = state.time + time;

		// Computing the CoM motion according to the projectile EoM
		current_state.com_pos = state.com_pos + state.com_vel * time +
				0.5 * gravity_vec * pow(time,2);
		current_state.com_vel = state.com_vel + gravity_vec * time;
		current_state.com_acc = gravity_vec;

		// Computing the heading motion by assuming that there isn't
		// change in the angular momentum
		current_state.head_pos = state.head_pos + state.head_vel * time;
		current_state.head_vel = state.head_vel;
		current_state.head_acc = 0.;

		// Appending the current state to the preview trajectory
		trajectory[k] = current_state;
	}
}


void PreviewLocomotion::addSwingPattern(PreviewTrajectory& trajectory,
										const PreviewState& state,
										const SwingParams& params)
{
	// Checking the preview duration
	if (params.duration < sample_time_)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time

	// Getting the number of samples of the trajectory
	unsigned int num_samples = ceil(params.duration / sample_time_);

	// Generating the feet trajectories
	feet_spline_generator_.clear();
	Eigen::Vector3d foot_pos, foot_vel, foot_acc;
	for (unsigned int k = 0; k < num_samples; k++) {
		double time = state.time + sample_time_ * (k + 1);
		if (time > state.time + params.duration)
			time = state.time + params.duration;

		// Generating the actual state for every feet
		for (rbd::BodyVector::const_iterator foot_it = state.foot_pos.begin();
				foot_it != state.foot_pos.end(); foot_it++) {
			std::string name = foot_it->first;

			// Checking the feet that swing
			rbd::BodyVector::const_iterator swing_it = params.feet_shift.find(name);
			if (swing_it != params.feet_shift.end()) {
				if (k == 0) { // Initialization of the swing generator
					// Getting the actual position of the contact w.r.t the CoM frame
					Eigen::Vector3d actual_pos = foot_it->second;

					// Getting the target position of the contact w.r.t the base
					Eigen::Vector3d target_pos;
					Eigen::Vector3d foot_shift = (Eigen::Vector3d) swing_it->second;
					Eigen::Vector3d stance_pos = actual_pos; // TODO read it
					target_pos = stance_pos + foot_shift;

					// Initializing the foot pattern generator
					simulation::StepParameters step_params(num_samples * sample_time_,
														   step_height_);
					feet_spline_generator_[name].setParameters(state.time,
															   actual_pos,
															   target_pos,
															   step_params);
				}

				// Generating the swing positions, velocities and accelerations
				feet_spline_generator_[name].generateTrajectory(foot_pos,
																foot_vel,
																foot_acc,
																time);

				// Adding the swing state to the trajectory
				trajectory[k].foot_pos[name] = foot_pos;
				trajectory[k].foot_vel[name] = foot_vel;
				trajectory[k].foot_acc[name] = foot_acc;
			} else {
				// There is not swing trajectory to generated (foot on ground).
				// Nevertheless, we have to updated their positions w.r.t the CoM frame
				// Getting the actual position of the contact w.r.t the CoM frame
				Eigen::Vector3d actual_pos = foot_it->second;

				// Getting the CoM position of the specific time
				Eigen::Vector3d com_pos = trajectory[k].com_pos;

				// Adding the foot states w.r.t. the CoM frame
				trajectory[k].foot_pos[name] = actual_pos - (com_pos - state.com_pos);
				trajectory[k].foot_vel[name] = Eigen::Vector3d::Zero();
				trajectory[k].foot_acc[name] = Eigen::Vector3d::Zero();
			}
		}
	}
}


void PreviewLocomotion::getPreviewTransitions(simulation::PreviewTrajectory& transitions,
											  const simulation::PreviewTrajectory& trajectory,
											  const simulation::PreviewControl& control)
{
	// Resizing the transitions
	unsigned int phases = control.params.size();
	transitions.resize(phases);

	// Getting the preview transitions
	int previous_index = -1;
	for (unsigned int k = 0; k < phases; k++) {
		// Getting the actual preview params
		PreviewParams params = control.params[k];

		// Checking the preview duration
		if (params.duration < sample_time_)
			continue; // duration it's always positive, and makes sense when
					  // is bigger than the sample time

		// Getting the index of the actual phase. Note that there is a
		// sanity check
		int index = previous_index +
				ceil(params.duration / sample_time_);
		if (index < 0)
			index = 0;

		// Setting the index as the previous one, which it will be used it for
		// the next for-iteration
		previous_index = index;

		// Adding the transition states. Note that CoP position and support
		// region are defined in the world frame
		transitions[k].time = trajectory[index].time;
		transitions[k].com_pos = trajectory[index].com_pos;
		transitions[k].com_vel = trajectory[index].com_vel;
		transitions[k].com_acc = trajectory[index].com_acc;
		transitions[k].cop = trajectory[index].cop;
		transitions[k].support_region = trajectory[index].support_region;
	}
}


model::FloatingBaseSystem* PreviewLocomotion::getFloatingBaseSystem()
{
	return &system_;
}


double PreviewLocomotion::getSampleTime()
{
	return sample_time_;
}


void PreviewLocomotion::toWholeBodyState(WholeBodyState& full_state,
										 const PreviewState& preview_state)
{
	// Adding the time
	full_state.time = preview_state.time;

	// From the preview model we do not know the joint states, so we neglect
	// the joint-related components of the CoM
	rbd::linearPart(full_state.base_pos) =
			preview_state.com_pos - actual_system_com_;
	rbd::linearPart(full_state.base_vel) = preview_state.com_vel;
	rbd::linearPart(full_state.base_acc) = preview_state.com_acc;

	full_state.base_pos(rbd::AZ) = preview_state.head_pos;
	full_state.base_vel(rbd::AZ) = preview_state.head_vel;
	full_state.base_acc(rbd::AZ) = preview_state.head_acc;

	// Adding the contact positions, velocities and accelerations
	// w.r.t the base frame
	for (rbd::BodyVector::const_iterator contact_it = preview_state.foot_pos.begin();
			contact_it != preview_state.foot_pos.end(); contact_it++) {
		std::string name = contact_it->first;
		full_state.contact_pos[name] = contact_it->second + actual_system_com_;
	}
	full_state.contact_vel = preview_state.foot_vel;
	full_state.contact_acc = preview_state.foot_acc;
}


void PreviewLocomotion::fromWholeBodyState(PreviewState& preview_state,
										   const WholeBodyState& full_state)
{
	// Adding the actual time
	preview_state.time = full_state.time;

	// Computing the CoM position, velocity and acceleration
	actual_system_com_ = system_.getSystemCoM(rbd::Vector6d::Zero(),
											  full_state.joint_pos);
	preview_state.com_pos = system_.getSystemCoM(full_state.base_pos,
												 full_state.joint_pos);
	preview_state.com_vel = system_.getSystemCoMRate(full_state.base_pos,
													 full_state.joint_pos,
													 full_state.base_vel,
													 full_state.joint_vel);
	// Neglecting the joint accelerations components
	preview_state.com_acc = full_state.base_acc.segment<3>(rbd::LX);

	preview_state.head_pos = full_state.base_pos(rbd::AZ);
	preview_state.head_vel = full_state.base_vel(rbd::AZ);
	preview_state.head_acc = full_state.base_acc(rbd::AZ);

	// Getting the world to base transformation
	Eigen::Vector3d base_traslation = full_state.base_pos.segment<3>(rbd::LX);
	Eigen::Vector3d base_rpy = full_state.base_pos.segment<3>(rbd::AX);
	Eigen::Matrix3d base_rotation = math::getRotationMatrix(base_rpy);

	// Computing the CoP in the world frame
	Eigen::Vector3d cop_wrt_base;
	dynamics_.computeCenterOfPressure(cop_wrt_base,
									  full_state.contact_eff,
									  full_state.contact_pos,
									  system_.getEndEffectorNames(model::FOOT));
	preview_state.cop = base_traslation + base_rotation * cop_wrt_base;

	// Getting the support region w.r.t the world frame. The support region
	// is defined by the active contacts
	rbd::BodySelector active_contacts;
	dynamics_.getActiveContacts(active_contacts,
								full_state.contact_eff,
								force_threshold_);
	preview_state.support_region.clear();
	for (unsigned int i = 0; i < active_contacts.size(); i++) {
		std::string name = active_contacts[i];

		preview_state.support_region[name] = base_traslation +
				full_state.contact_pos.find(name)->second;
	}

	// Adding the contact positions, velocities and accelerations
	// w.r.t the CoM frame
	for (rbd::BodyVector::const_iterator contact_it = full_state.contact_pos.begin();
			contact_it != full_state.contact_pos.end(); contact_it++) {
		std::string name = contact_it->first;
		preview_state.foot_pos[name] = contact_it->second - actual_system_com_;
	}
	preview_state.foot_vel = full_state.contact_vel;
	preview_state.foot_acc = full_state.contact_acc;
}


void PreviewLocomotion::toWholeBodyTrajectory(WholeBodyTrajectory& full_traj,
											  const PreviewTrajectory& preview_traj)
{
	// Getting the number of points defined in the preview trajectory
	unsigned int traj_size = preview_traj.size();

	// Resizing the full trajectory vector
	full_traj.clear();
	full_traj.resize(traj_size);

	// Getting the full trajectory
	dwl::WholeBodyState full_state;
	for (unsigned int k = 0; k < traj_size; k++) {
		toWholeBodyState(full_state, preview_traj[k]);

		full_traj[k] = full_state;
	}
}

} //@namespace simulation
} //@namespace dwl
