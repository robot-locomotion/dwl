#include <dwl/simulation/PreviewLocomotion.h>


namespace dwl
{

namespace simulation
{

PreviewLocomotion::PreviewLocomotion() : robot_model_(false),
		sample_time_(0.001), gravity_(9.81), mass_(0.), num_feet_(0),
		step_height_(0.1)
{

}


PreviewLocomotion::~PreviewLocomotion()
{

}


void PreviewLocomotion::resetFromURDFFile(std::string urdf_file,
										  std::string system_file)
{
	resetFromURDFModel(urdf_model::fileToXml(urdf_file), system_file);
}


void PreviewLocomotion::resetFromURDFModel(std::string urdf_model,
										   std::string system_file)
{
	// Initializing the dynamics, kinematics and system from the URDF model
	wdyn_.modelFromURDFModel(urdf_model, system_file);
	wkin_ = wdyn_.getWholeBodyKinematics();
	fbs_ = wdyn_.getFloatingBaseSystem();

	// Resetting the states converter
	state_tf_.reset(wdyn_);

	// Getting the gravity magnitude from the rigid-body dynamic model
	gravity_ = fbs_.getRBDModel().gravity.norm();

	// Getting the total mass of the system
	mass_ = fbs_.getTotalMass();

	// Getting the number of feet
	num_feet_ = fbs_.getNumberOfEndEffectors(model::FOOT);

	// Getting the feet names
	feet_names_ = fbs_.getEndEffectorNames(model::FOOT);

	// Getting the default joint position
	Eigen::VectorXd q0 = fbs_.getDefaultPosture();

	// Getting the default position of the CoM system w.r.t. the base frame
//	Eigen::Vector3d com_pos_B = fbs_.getSystemCoM(rbd::Vector6d::Zero(), q0);

	// Computing the stance posture using the default position
	wkin_.computeForwardKinematics(stance_posture_H_,
								   rbd::Vector6d::Zero(),
								   q0,
								   feet_names_,
								   rbd::Linear);

	// Converting to the CoM frame //TODO remove for testing
//	for (rbd::BodyVectorXd::iterator feet_it = stance_posture_C_.begin();
//			feet_it != stance_posture_C_.end(); feet_it++) {
//		std::string name = feet_it->first;
//		Eigen::VectorXd stance = feet_it->second;
//
//		stance_posture_C_[name] = stance - com_pos_B;
//	}

	// Setting up the cart-table model
	CartTableProperties model(mass_, gravity_);
	cart_table_.setModelProperties(model);

	robot_model_ = true;
}


void PreviewLocomotion::readPreviewSequence(PreviewData& data,
											std::string filename)
{
	YamlWrapper yaml_reader(filename);
	YamlNamespace datapoint_ns = {"preview_sequence"};

	// Reading the number of datapoint
	int num_datapoint;
	if (!yaml_reader.read(num_datapoint, "number_datapoint", datapoint_ns)) {
		printf(RED "Error: the number of datapoint was not found\n" COLOR_RESET);
		return;
	}

	// Reading the preview sequence
	data.resize(num_datapoint);
	for (int k = 0; k < num_datapoint; ++k) {
		YamlNamespace data_ns = {"preview_sequence",
								 "datapoint_" + std::to_string(k)};

		// Reading the actual preview data point
		readPreviewSequence(data[k].command,
							data[k].state,
							data[k].control,
							filename,
							data_ns);
	}
}


void PreviewLocomotion::readPreviewSequence(VelocityCommand& command,
											PreviewState& state,
											PreviewControl& control,
											std::string filename,
											 YamlNamespace seq_ns)
{
	// Checking that the robot model was initialized
	if (!robot_model_) {
		printf(RED "Error: the robot model was not initialized\n" COLOR_RESET);
		return;
	}

	YamlWrapper yaml_reader(filename);

	// All the preview sequence data have to be inside the state and
	// preview_control namespaces
	YamlNamespace command_ns = seq_ns;
	YamlNamespace state_ns = seq_ns;
	YamlNamespace control_ns = seq_ns;
	command_ns.push_back("command");
	state_ns.push_back("state");
	control_ns.push_back("preview_control");

	// Reading the command
	if (!yaml_reader.read(command.linear, "linear", command_ns)) {
		printf(RED "Error: the linear velocity was not found\n" COLOR_RESET);
		return;
	}
	if (!yaml_reader.read(command.angular, "angular", command_ns)) {
		printf(RED "Error: the angular velocity was not found\n" COLOR_RESET);
		return;
	}


	// Reading the state
	if (!yaml_reader.read(state.height, "height", state_ns)) {
		printf(RED "Error: the CoM height was not found\n" COLOR_RESET);
		return;
	}
	if (!yaml_reader.read(state.com_pos, "com_pos", state_ns)) {
		printf(RED "Error: the CoM position was not found\n" COLOR_RESET);
		return;
	}
	if (!yaml_reader.read(state.com_vel, "com_vel", state_ns)) {
		printf(RED "Error: the CoM velocity was not found\n" COLOR_RESET);
		return;
	}
	std::vector<std::string> support;
	if (!yaml_reader.read(support, "support", state_ns)) {
		printf(RED "Error: the support was not found\n" COLOR_RESET);
		return;
	}
	state.support.clear();
	for (unsigned int f = 0; f < support.size(); f++)
		state.support[support[f]] = true;

	// Reading the preview control data
	// Reading the number of phases
	int num_phases;
	if (!yaml_reader.read(num_phases, "number_phase",  control_ns)) {
		printf(RED "Error: the number_phase was not found\n" COLOR_RESET);
		return;
	}
	control.params.resize(num_phases);

	// Reading the preview parameters per phase
	for (int k = 0; k < num_phases; ++k) {
		// Getting the phase namespace
		YamlNamespace phase_ns = control_ns;
		phase_ns.push_back("phase_" + std::to_string(k));

		// Reading the preview duration
		if (!yaml_reader.read(control.params[k].duration, "duration", phase_ns)) {
			printf(RED "Error: the duration of phase_%i was not found\n"
					COLOR_RESET, k);
			return;
		}

		// Reading the preview CoP shift
		if (yaml_reader.read(control.params[k].cop_shift, "cop_shift", phase_ns))
			control.params[k].phase.setTypeOfPhase(simulation::STANCE);

		// Reading the footstep shifts
		for (unsigned int f = 0; f < feet_names_.size(); ++f) {
			std::string name = feet_names_[f];

			// Setting up if there is a foot shift in this phase
			Eigen::Vector2d foot_shift;
			if (yaml_reader.read(foot_shift, name, phase_ns)) {
				control.params[k].phase.feet.push_back(name);
				control.params[k].phase.setSwingFoot(name);
				control.params[k].phase.setFootShift(name, foot_shift);
			}
		}
	}
}


void PreviewLocomotion::setSampleTime(double sample_time)
{
	sample_time_ = sample_time;
}


void PreviewLocomotion::setStepHeight(double step_height)
{
	step_height_ = step_height;
}


void PreviewLocomotion::setForceThreshold(double force_threshold)
{
	state_tf_.setForceThreshold(force_threshold);
}


void PreviewLocomotion::multiPhasePreview(ReducedBodyTrajectory& trajectory,
										  const ReducedBodyState& state,
										  const PreviewControl& control,
										  bool full)
{
	// Checking that the robot model was initialized
	if (!robot_model_) {
		printf(RED "Error: the robot model was not initialized\n" COLOR_RESET);
		return;
	}

	// Updating the actual state
	actual_state_ = state;

	// Clearing the trajectory
	trajectory.clear();

	// Computing the preview for multi-phase
	ReducedBodyState initial_state = state;
	unsigned int num_phases = control.params.size();
	for (unsigned int k = 0; k < num_phases; ++k) {
		ReducedBodyTrajectory phase_traj;

		// Getting the preview params of the actual phase
		const PreviewParams& preview_params = control.params[k];

		// Computing the preview of the actual phase
		if (preview_params.phase.type == STANCE) {
			stancePreview(phase_traj, initial_state, preview_params, full);
		} else {
			flightPreview(phase_traj, initial_state, preview_params, full);
		}

		// Appending the actual phase trajectory
		trajectory.insert(trajectory.end(), phase_traj.begin(), phase_traj.end());

		// Sanity action: defining the actual state if there isn't a trajectory
		if (trajectory.size() == 0)
			trajectory.push_back(state);

		// Updating the actual preview state for the next phase
		initial_state = trajectory.back();
	}
}


void PreviewLocomotion::multiPhaseEnergy(Eigen::Vector3d& com_energy,
										 const ReducedBodyState& state,
										 const PreviewControl& control)
{
	// Checking that the robot model was initialized
	if (!robot_model_) {
		printf(RED "Error: the robot model was not initialized\n" COLOR_RESET);
		return;
	}

	// Updating the actual state
	actual_state_ = state;

	// Initializing the CoM energy vector
	com_energy.setZero();

	// Computing the energy for multi-phase
	ReducedBodyState initial_state = state;
	for (unsigned int k = 0; k < control.params.size(); ++k) {
		// Getting the preview params of the actual phase
		PreviewParams preview_params = control.params[k];


		// Computing the CoM energy of this phase
		if (preview_params.phase.type == STANCE) {
			Eigen::Vector3d phase_energy;
			CartTableControlParams model_params(preview_params.duration,
											    preview_params.cop_shift);
			cart_table_.computeSystemEnergy(phase_energy,
											initial_state,
											model_params);
			com_energy += phase_energy;
		} else { // Flight phase
			// TODO compute the energy for flight phases
		}

		// Updating the actual state
		double time = initial_state.time + preview_params.duration;
		cart_table_.computeResponse(initial_state, time);
	}
}


void PreviewLocomotion::stancePreview(ReducedBodyTrajectory& trajectory,
									  const ReducedBodyState& state,
									  const PreviewParams& params,
									  bool full)
{
	// Checking the preview duration
	if (full && params.duration < sample_time_)
		return; // duration it's always positive, and makes sense when
				// is bigger than the sample time

	// Adding the actual support region. Note that the support region
	// remains constant during this phase
	ReducedBodyState current_state = state;
	for (unsigned int f = 0; f < num_feet_; ++f) {
		std::string name = feet_names_[f];

		// Removing the swing foot of the actual phase
		if (params.phase.isSwingFoot(name))
			current_state.support_region.erase(name);
	}

	// Initialization of the Linear Controlled SLIP model
	CartTableControlParams model_params(params.duration,
										params.cop_shift);
	cart_table_.initResponse(current_state, model_params);

	// Computing the number of samples and initial index
	unsigned int num_samples = floor(params.duration / sample_time_);
	unsigned int idx;
	if (full) {
		idx = 0;
		trajectory.resize(num_samples + 1);

		// Initialization of the swing generator
		initSwing(current_state, params);
	} else {
		idx = num_samples;
		trajectory.resize(2);
	}

	// Computing the preview trajectory
	double time;
	for (unsigned int k = idx; k < num_samples + 1; ++k) {
		// Computing the current time of the preview trajectory
		if (k == num_samples)
			time = params.duration;
		else
			time = sample_time_ * (k + 1);
		current_state.time = state.time + time;

		// Computing the response of the Linear Controlled SLIP
		// dynamics
		cart_table_.computeResponse(current_state,
									current_state.time);

		// Generating the swing trajectory
		if (full)
			generateSwing(current_state, current_state.time);

		// Appending the current state to the preview trajectory
		trajectory[k-idx] = current_state;
	}

	// Adding the foothold target of the previous phase
	trajectory[1] = trajectory[0];
	for (unsigned int f = 0; f < num_feet_; ++f) {
		std::string name = feet_names_[f];
		if (params.phase.isSwingFoot(name)) {
			Eigen::Vector3d stance_H =
					stance_posture_H_.find(name)->second;

			// Getting the footshift control parameter
			Eigen::Vector2d footshift_2d =
					params.phase.getFootShift(name);
			Eigen::Vector3d footshift_H(footshift_2d(rbd::X),
										footshift_2d(rbd::Y),
										0.);

			// Computing the foothold position w.r.t. the world.
			// Note that the footshift is always expressed in the
			// horizontal frame
			Eigen::Vector3d foothold =
					trajectory.back().com_pos + stance_H + footshift_H;

			if (terrain_.isTerrainInformation()) {
				// Adding the terrain height given the terrain
				// height-map
				Eigen::Vector2d foothold_2d = foothold.head<2>();
				foothold(rbd::Z) = terrain_.getTerrainHeight(foothold_2d);
			} else {
				foothold(rbd::Z) = actual_state_.com_pos(rbd::Z) -
					cart_table_.getPendulumHeight();
			}

			trajectory.back().support_region[name] = foothold;
		}
	}
}


void PreviewLocomotion::flightPreview(ReducedBodyTrajectory& trajectory,
						   	   	   	  const ReducedBodyState& state,
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
	unsigned int num_samples = floor(params.duration / sample_time_);
	unsigned int idx;
	if (full) {
		idx = 0;
		trajectory.resize(num_samples);

		// Initialization of the swing generator
		initSwing(state, params);
	} else {
		idx = num_samples;
		trajectory.resize(1);
	}

	// Computing the preview trajectory
	double time;
	for (unsigned int k = idx; k < num_samples + 1; ++k) {
		// Computing the current time of the preview trajectory
		if (k == num_samples)
			time = params.duration;
		else
			time = sample_time_ * (k + 1);


		// Computing the current time of the preview trajectory
		ReducedBodyState current_state;
		current_state.time = state.time + time;

		// Computing the CoM motion according to the projectile EoM //TODO these are in the H frame
		current_state.com_pos = state.com_pos + state.com_vel * time +
				0.5 * gravity_vec * time * time;
		current_state.com_vel = state.com_vel + gravity_vec * time;
		current_state.com_acc = gravity_vec;

		// Generating the swing trajectory
		if (full)
			generateSwing(current_state, current_state.time);

		// Appending the current state to the preview trajectory
		trajectory[k-idx] = current_state;
	}
}



void PreviewLocomotion::initSwing(const ReducedBodyState& state,
								  const PreviewParams& params)
{
	// Updating the phase state
	phase_state_ = state;

	// Computing the terminal CoM state for getting the foothold position
	ReducedBodyState terminal_state;
	cart_table_.computeResponse(terminal_state,
								state.time + params.duration);

	// Getting the swing shift per foot
	rbd::BodyVector3d swing_shift_B;
	for (unsigned int j = 0; j < params.phase.feet.size(); ++j) {
		std::string name = params.phase.feet[j];
		Eigen::Vector3d stance_H = stance_posture_H_.find(name)->second;

		// Getting the footshift control parameter
		Eigen::Vector2d footshift_2d = params.phase.getFootShift(name);
		Eigen::Vector3d footshift_H(footshift_2d(rbd::X),
									footshift_2d(rbd::Y),
									0.);

		// Computing the foothold position w.r.t. the world. Note that the
		// footshift is always expressed in the horizontal frame
		Eigen::Vector3d foothold =
				terminal_state.com_pos + stance_H + footshift_H;

		// Computing the footshift in z from the height map. In case of no
		// having the terrain height map, it assumes flat terrain conditions.
		// Note that, for those cases, we compensate small drift between the
		// actual and the default postures, and the displacement of the CoM in z
		if (terrain_.isTerrainInformation()) {
			// Getting the terminal CoM position in the horizontal frame
			Eigen::Vector3d terminal_com_pos_H =
					frame_tf_.fromWorldToHorizontalFrame(terminal_state.getCoMPosition_W(),
														 terminal_state.getRPY_W());

			// Adding the terrain height given the terrain height-map
			Eigen::Vector2d foothold_2d = foothold.head<2>();
			footshift_H(rbd::Z) = terrain_.getTerrainHeight(foothold_2d) -
					(terminal_com_pos_H(rbd::Z) + stance_H(rbd::Z));
		} else {
			// Computing the nominal CoM height that considers terrain elevations
			Eigen::Vector3d height(0., 0., cart_table_.getPendulumHeight());
			Eigen::Vector3d nominal_com_pos =
					height + (terminal_state.com_pos - actual_state_.com_pos);
			double nominal_com_height =
					frame_tf_.fromWorldToHorizontalFrame(nominal_com_pos,
														 terminal_state.getRPY_W())(rbd::Z);
			footshift_H(rbd::Z) = -(nominal_com_height + stance_H(rbd::Z));
		}

		swing_shift_B[name] =
				frame_tf_.fromHorizontalToBaseFrame(footshift_H,
													terminal_state.getRPY_W());
	}

	// Adding the swing pattern expressed in the CoM frame
	swing_params_ = SwingParams(params.duration, swing_shift_B);

	// Generating the actual state for every feet
	feet_spline_generator_.clear();
	for (rbd::BodyVector3d::const_iterator foot_it = state.foot_pos.begin();
			foot_it != state.foot_pos.end(); ++foot_it) {
		std::string name = foot_it->first;

		// Checking the feet that swing
		rbd::BodyVector3d::const_iterator swing_it = swing_params_.feet_shift.find(name);
		if (swing_it != swing_params_.feet_shift.end()) {
			// Getting the actual position of the contact w.r.t the CoM frame
			Eigen::Vector3d actual_pos_B = state.getFootPosition_B(foot_it);

			// Getting the target position of the contact w.r.t the CoM frame
			Eigen::Vector3d footshift_B = (Eigen::Vector3d) swing_it->second;
			Eigen::Vector3d stance_pos_H = stance_posture_H_.find(name)->second.head<3>();
			Eigen::Vector3d stance_pos_B =
					frame_tf_.fromHorizontalToBaseFrame(stance_pos_H,
														terminal_state.getRPY_W());
			Eigen::Vector3d target_pos_B = stance_pos_B + footshift_B;

			// Initializing the foot pattern generator
			double penetration = 0.;
			simulation::StepParameters step_params(params.duration,//num_samples * sample_time_,
												   step_height_, penetration);// TODO read it
			feet_spline_generator_[name].setParameters(state.time,
													   actual_pos_B,
													   target_pos_B,
													   step_params);
		}
	}
}


void PreviewLocomotion::generateSwing(ReducedBodyState& state,
									  double time)
{
	// Generating the actual state for every feet
	Eigen::Vector3d foot_pos_B, foot_vel_B, foot_acc_B;
	for (rbd::BodyVector3d::const_iterator foot_it = phase_state_.foot_pos.begin();
			foot_it != phase_state_.foot_pos.end(); ++foot_it) {
		std::string name = foot_it->first;

		// Checking the feet that swing
		rbd::BodyVector3d::const_iterator swing_it = swing_params_.feet_shift.find(name);
		if (swing_it != swing_params_.feet_shift.end()) {
			// Generating the swing positions, velocities and accelerations
			// expressed in the CoM frame
			feet_spline_generator_[name].generateTrajectory(foot_pos_B,
															foot_vel_B,
															foot_acc_B,
															time);

			// Adding the swing state to the trajectory
			state.setFootPosition_B(name, foot_pos_B);
			state.setFootVelocity_B(name, foot_vel_B);
			state.setFootAcceleration_B(name, foot_acc_B);
		} else {
			// There is not swing trajectory to generated (foot on ground).
			// Nevertheless, we have to updated their positions w.r.t the CoM frame
			// Getting the actual position of the foot expressed in the horizontal
			// frame
			Eigen::Vector3d actual_pos_H = phase_state_.getFootPosition_H(foot_it);

			// Getting the CoM position of the specific time
			Eigen::Vector3d com_pos = state.com_pos;
			Eigen::Vector3d com_vel = state.com_vel;
			Eigen::Vector3d com_acc = state.com_acc;

			// Adding the foot states w.r.t. the CoM frame
			Eigen::Vector3d com_disp_W = com_pos - phase_state_.com_pos;
			Eigen::Vector3d com_disp_H =
					frame_tf_.fromWorldToHorizontalFrame(com_disp_W, state.getRPY_W());
			state.setFootPosition_H(name, actual_pos_H - com_disp_H);
			state.setFootVelocity_H(name, -com_vel);
			state.setFootAcceleration_H(name, -com_acc);
		}
	}
}


model::FloatingBaseSystem* PreviewLocomotion::getFloatingBaseSystem()
{
	return &fbs_;
}


model::WholeBodyDynamics* PreviewLocomotion::getWholeBodyDynamics()
{
	return &wdyn_;
}


environment::TerrainMap* PreviewLocomotion::getTerrainMap()
{
	return &terrain_;
}


double PreviewLocomotion::getSampleTime()
{
	return sample_time_;
}


void PreviewLocomotion::toWholeBodyState(WholeBodyState& full_state,
										 const ReducedBodyState& reduced_state)
{
	full_state = state_tf_.getWholeBodyState(reduced_state);
}


void PreviewLocomotion::fromWholeBodyState(ReducedBodyState& reduced_state,
										   const WholeBodyState& full_state)
{
	reduced_state = state_tf_.getReducedBodyState(full_state);
}


void PreviewLocomotion::toWholeBodyTrajectory(WholeBodyTrajectory& full_traj,
											  const ReducedBodyTrajectory& reduced_traj)
{
	full_traj = state_tf_.getWholeBodyTrajectory(reduced_traj);
}

} //@namespace simulation
} //@namespace dwl
