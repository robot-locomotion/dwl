#include <dwl/simulation/PreviewLocomotion.h>


namespace dwl
{

namespace simulation
{

PreviewLocomotion::PreviewLocomotion() : sample_time_(0.001), gravity_(9.81),
		mass_(0.), num_feet_(0), step_height_(0.1), force_threshold_(0.)
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

	// Initializing the dynamics and kinematics from the URDF model
	dynamics_.modelFromURDFModel(urdf_model, system_file);
	kinematics_.modelFromURDFModel(urdf_model, system_file);

	// Setting the gravity magnitude from the rigid-body dynamic model
	gravity_ = system_.getRBDModel().gravity.norm();

	// Getting the total mass of the system
	mass_ = system_.getTotalMass();

	// Getting the number of feet
	num_feet_ = system_.getNumberOfEndEffectors(model::FOOT);

	// Getting the feet names
	feet_names_ = system_.getEndEffectorNames(model::FOOT);

	// Getting the floating-base CoM
	actual_system_com_ = system_.getFloatingBaseCoM();


	stance_posture_["lf_foot"] << 0.36, 0.32, -0.55;
	stance_posture_["rf_foot"] << 0.36, -0.32, -0.55;
	stance_posture_["lh_foot"] << -0.36, 0.32, -0.55;
	stance_posture_["rh_foot"] << -0.36, -0.32, -0.55;
}


void PreviewLocomotion::setSampleTime(double sample_time)
{
	sample_time_ = sample_time;
}


void PreviewLocomotion::setStiffnes(double stiffnes)
{
	SlipProperties model(mass_, stiffnes, gravity_);
	lc_slip_.setModelProperties(model);
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
										  const PreviewControl& control,
										  bool full)
{
	// Clearing the trajectory
	trajectory.clear();

	// Computing the preview for multi-phase
	PreviewState actual_state;
	rbd::BodyPosition last_suppport_region;
	for (unsigned int k = 0; k < control.params.size(); k++) {
		PreviewTrajectory phase_traj;

		// Getting the preview params of the actual phase
		PreviewParams preview_params = control.params[k];

		// Getting the actual preview state for this phase
		if (k == 0)
			actual_state = state;
		else {
			actual_state = trajectory.back();

			// Updating the support region for this phase
			if (preview_params.duration > sample_time_) {
				for (unsigned int f = 0; f < num_feet_; f++) {
					std::string name = feet_names_[f];

					// Removing the swing foot of the actual phase
					if (preview_params.phase.isSwingFoot(name)) {
						last_suppport_region[name] = actual_state.support_region.find(name)->second;
						actual_state.support_region.erase(name);
					}

					// Adding the foothold target of the previous phase
					if (control.params[k-1].phase.isSwingFoot(name) &&
							control.params[k-1].duration > sample_time_) {
						// Computing the target foothold of the contact w.r.t the world frame
						Eigen::Vector2d foot_2d_shift = control.params[k-1].phase.getFootShift(name);
						Eigen::Vector3d foot_shift(foot_2d_shift(rbd::X), foot_2d_shift(rbd::Y), 0.);
						Eigen::Vector3d stance_pos;
						stance_pos << stance_posture_.find(name)->second.head<2>(),
									  last_suppport_region.find(name)->second(2);

						// Computing the foothold target position
						Eigen::Vector3d planar_com_pos(actual_state.com_pos(rbd::X),
													   actual_state.com_pos(rbd::Y),
													   0.);
						Eigen::Vector3d next_foothold = planar_com_pos + stance_pos + foot_shift;

						actual_state.support_region[name] = next_foothold;
					}
				}
			}
		}

		// Computing the preview of the actual phase
		if (preview_params.phase.type == STANCE) {
			stancePreview(phase_traj, actual_state, preview_params, full);

			// Computing the swing trajectories for full cases
			if (full) {
				// Getting the swing shift per foot
				rbd::BodyPosition swing_shift;
				for (unsigned int j = 0; j < preview_params.phase.feet.size(); j++) {
					std::string foot_name = preview_params.phase.feet[j];
					Eigen::Vector2d foot_shift_2d =
							control.params[k-1].phase.getFootShift(foot_name);

					// Computing the z displacement of the foot from the height map. TODO hard coded
//					Eigen::Vector3d terminal_base_pos = phase_traj.end()->com_pos - actual_system_com_;
//					Eigen::Vector2d foothold_2d = foot_shift_2d + terminal_base_pos.head<2>();
					double z_shift = 0.;

					Eigen::Vector3d foot_shift(foot_shift_2d(dwl::rbd::X),
											   foot_shift_2d(dwl::rbd::Y),
											   z_shift);
					swing_shift[foot_name] = foot_shift;
				}

				// Adding the swing pattern
				SwingParams swing_params(preview_params.duration, swing_shift);
				addSwingPattern(phase_traj, actual_state, swing_params);
			}
		} else {
			flightPreview(phase_traj, actual_state, preview_params, full);

			// Computing the swing trajectories for full cases
			if (full) {
				// Adding the swing pattern
				SwingParams swing_params(preview_params.duration,
										 rbd::BodyPosition()); // no foothold targets
				addSwingPattern(phase_traj, actual_state, swing_params);
			}
		}

		// Appending the actual phase trajectory
		trajectory.insert(trajectory.end(), phase_traj.begin(), phase_traj.end());

		// Sanity action: defining the actual state if there isn't a trajectory
		if (trajectory.size() == 0)
			trajectory.push_back(state);
	}
}


void PreviewLocomotion::multiPhaseEnergy(Eigen::Vector3d& com_energy,
										 const PreviewState& state,
										 const PreviewControl& control)
{
	// Initializing the CoM energy vector
	com_energy.setZero();

	// Computing the energy for multi-phase
	PreviewState actual_state = state;
	rbd::BodyPosition last_suppport_region;
	for (unsigned int k = 0; k < control.params.size(); k++) {
		// Getting the preview params of the actual phase
		PreviewParams preview_params = control.params[k];


		// Computing the CoM energy of this phase
		if (preview_params.phase.type == STANCE) {
			Eigen::Vector3d phase_energy;
			ReducedBodyState reduced_state(actual_state.time,
										   actual_state.com_pos,
										   actual_state.com_vel,
										   actual_state.com_acc,
										   actual_state.cop);
			SlipControlParams slip_params(preview_params.duration,
										  preview_params.cop_shift,
										  preview_params.length_shift);
			lc_slip_.computeSystemEnergy(phase_energy,
										 reduced_state,
										 slip_params);
			com_energy += phase_energy;
		} else { // Flight phase
			// TODO compute the energy for flight phases
		}

		// Updating the actual state
		ReducedBodyState next_reduced_state;
		double time = actual_state.time + preview_params.duration;
		lc_slip_.computeResponse(next_reduced_state, time);
		actual_state.time = next_reduced_state.time;
		actual_state.com_pos = next_reduced_state.com_pos;
		actual_state.com_vel = next_reduced_state.com_vel;
		actual_state.com_acc = next_reduced_state.com_acc;
		actual_state.cop = next_reduced_state.cop;
	}
}


void PreviewLocomotion::stancePreview(PreviewTrajectory& trajectory,
									  const PreviewState& state,
									  const PreviewParams& params,
									  bool full)
{
	// Checking the preview duration
	if (full && params.duration < sample_time_)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time

	// Initialization of the Linear Controlled SLIP model
	ReducedBodyState reduced_state(state.time,
								   state.com_pos,
								   state.com_vel,
								   state.com_acc,
								   state.cop);
	SlipControlParams slip_params(params.duration,
								  params.cop_shift,
								  params.length_shift);
	lc_slip_.initResponse(reduced_state, slip_params);

	// Adding the actual support region. Note that the support region
	// remains constant during this phase
	PreviewState current_state;
	current_state.support_region = state.support_region;

	// Computing the number of samples and initial index
	unsigned int num_samples = ceil(params.duration / sample_time_);
	unsigned int idx;
	if (full) {
		idx = 0;
		trajectory.resize(num_samples);
	} else {
		idx = num_samples - 1;
		trajectory.resize(1);
	}

	// Computing the preview trajectory
	for (unsigned int k = idx; k < num_samples; k++) {
		// Computing the current time of the preview trajectory
		double time = sample_time_ * (k + 1);
		current_state.time = state.time + time;

		// Computing the response of the Linear Controlled SLIP
		// dynamics
		ReducedBodyState reduced_state;
		lc_slip_.computeResponse(reduced_state,
								 current_state.time);
		current_state.com_pos = reduced_state.com_pos;
		current_state.com_vel = reduced_state.com_vel;
		current_state.com_acc = reduced_state.com_acc;
		current_state.cop = reduced_state.cop;

		// Computing the heading motion according to heading kinematic equation
		current_state.head_pos = state.head_pos + state.head_vel * time +
				0.5 * params.head_acc * pow(time,2);
		current_state.head_vel = state.head_vel + params.head_acc * time;
		current_state.head_acc = params.head_acc;

		// Appending the current state to the preview trajectory
		trajectory[k-idx] = current_state;
	}
}


void PreviewLocomotion::flightPreview(PreviewTrajectory& trajectory,
						   	   	   	  const PreviewState& state,
									  const PreviewParams& params,
									  bool full)
{
	// Checking the preview duration
	if (full && params.duration < sample_time_)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time

	// Setting the gravity vector
	Eigen::Vector3d gravity_vec = Eigen::Vector3d::Zero();
	gravity_vec(rbd::Z) = -gravity_;

	// Computing the number of samples and initial index
	unsigned int num_samples = ceil(params.duration / sample_time_);
	unsigned int idx;
	if (full) {
		idx = 0;
		trajectory.resize(num_samples);
	} else {
		idx = num_samples - 1;
		trajectory.resize(1);
	}

	// Computing the preview trajectory
	for (unsigned int k = idx; k < num_samples; k++) {
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
		trajectory[k-idx] = current_state;
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
			rbd::BodyPosition::const_iterator swing_it = params.feet_shift.find(name);
			if (swing_it != params.feet_shift.end()) {
				if (k == 0) { // Initialization of the swing generator
					// Getting the actual position of the contact w.r.t the CoM frame
					Eigen::Vector3d actual_pos = foot_it->second;

					// Getting the target position of the contact w.r.t the base
					Eigen::Vector3d target_pos;
					Eigen::Vector3d foot_shift = (Eigen::Vector3d) swing_it->second;
					Eigen::Vector3d stance_pos;
					stance_pos << stance_posture_.find(name)->second.head<2>(), actual_pos(rbd::Z); // TODO read it
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


model::FloatingBaseSystem* PreviewLocomotion::getFloatingBaseSystem()
{
	return &system_;
}


model::WholeBodyDynamics* PreviewLocomotion::getWholeBodyDynamics()
{
	return &dynamics_;
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

	full_state.base_pos(rbd::AZ) = 0.;//preview_state.head_pos;//TODO for debugging
	full_state.base_vel(rbd::AZ) = 0.;//preview_state.head_vel;
	full_state.base_acc(rbd::AZ) = 0.;//preview_state.head_acc;


	// Adding the contact positions, velocities and accelerations
	// w.r.t the base frame
	for (rbd::BodyVector::const_iterator contact_it = preview_state.foot_pos.begin();
			contact_it != preview_state.foot_pos.end(); contact_it++) {
		std::string name = contact_it->first;
		full_state.contact_pos[name] = contact_it->second + actual_system_com_;
	}
	full_state.contact_vel = preview_state.foot_vel;
	full_state.contact_acc = preview_state.foot_acc;

	// Adding infinity contact force for active feet
	for (unsigned int f = 0; f < num_feet_; f++) {
		std::string name = feet_names_[f];

		rbd::BodyPosition::const_iterator support_it = preview_state.support_region.find(name);
		if (support_it != preview_state.support_region.end())
			full_state.contact_eff[name] = ACTIVE_CONTACT;
		else
			full_state.contact_eff[name] = INACTIVE_CONTACT;
	}


	// Adding the joint positions, velocities and accelerations
	dwl::rbd::BodyPosition feet_pos;
	full_state.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	full_state.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	full_state.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	for (unsigned int f = 0; f < num_feet_; f++) {
		std::string name = feet_names_[f];

		feet_pos[name] = preview_state.foot_pos.find(name)->second;
	}

	// Computing the joint positions
	kinematics_.computeInverseKinematics(full_state.joint_pos,
										 feet_pos);

	// Computing the joint velocities
	kinematics_.computeJointVelocity(full_state.joint_vel,
									 full_state.joint_pos,
									 full_state.contact_vel,
									 feet_names_);

	// Computing the joint accelerations
	kinematics_.computeJoinAcceleration(full_state.joint_acc,
										full_state.joint_pos,
										full_state.joint_vel,
										full_state.contact_vel,
										feet_names_);

	// Setting up the desired joint efforts equals to zero
	full_state.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());
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
									  feet_names_);
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
