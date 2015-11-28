#include <dwl/simulation/PreviewLocomotion.h>


namespace dwl
{

namespace simulation
{

PreviewLocomotion::PreviewLocomotion() : sample_time_(0.001), gravity_(9.81), mass_(0.)
{

}


PreviewLocomotion::~PreviewLocomotion()
{

}


void PreviewLocomotion::resetFromURDFFile(std::string filename)
{
	// Reading the file
	std::ifstream model_file(filename.c_str());
	if (!model_file) {
		std::cerr << "Error opening file '" << filename << "'." << std::endl;
		abort();
	}

	// Reserving memory for the contents of the file
	std::string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
			std::istreambuf_iterator<char>());
	model_file.close();

	resetFromURDFFile(model_xml_string);
}


void PreviewLocomotion::resetFromURDFModel(std::string urdf_model)
{
	system_.resetFromURDFModel(urdf_model);

	// Setting the gravity magnitude from the rigid-body dynamic model
	gravity_ = system_.getRBDModel().gravity.norm();

	// Getting the total mass of the system
	mass_ = system_.getTotalMass();
}


void PreviewLocomotion::setSampleTime(double sample_time)
{
	sample_time_ = sample_time;
}

/*
void PreviewLocomotion::previewScheduled(PreviewTrajectory& trajectory,
										 std::vector<QuadrupedalPreviewParameters> control_params)
{
	unsigned int num_phases = control_params.size();
	for (unsigned int i = 0; i < num_phases; i++) {
		// Computing the number of samples
		double phase_duration = control_params[i].four_support.duration;
		unsigned int num_samples = round(phase_duration / sample_time_);

		for (unsigned int k = 0; k < num_samples; k++) {
			double current_time = sample_time_ * k;

			// Computing the preview locomotion trajectory
			WholeBodyState current_state;
			stancePreview(current_state, current_time);

			trajectory.push_back(current_state);
		}

	}
}
*/

void PreviewLocomotion::stancePreview(PreviewTrajectory& trajectory,
									  const PreviewState& initial_state,
									  const StancePreviewParameters& params)
{
	// Computing the coefficients of the SLIP response
	double pendulum_height = 0.58; //TODO set it
	double slip_omega = sqrt(gravity_ / pendulum_height);
	double alpha = 2 * slip_omega * params.duration;
	Eigen::Vector2d slip_hor_proj = initial_state.com_pos.head<2>() - params.initial_cop;
	Eigen::Vector2d cop_disp = params.initial_cop - params.terminal_cop;
	Eigen::Vector2d slip_hor_disp = initial_state.com_vel.head<2>() * params.duration;
	Eigen::Vector2d beta_1 = slip_hor_proj / 2 + (slip_hor_disp - cop_disp) / alpha;
	Eigen::Vector2d beta_2 = slip_hor_proj / 2 - (slip_hor_disp - cop_disp) / alpha;

	// Computing the coefficients of the spring-mass system response
	double spring_gain = 400.; //TODO set it
	double spring_omega = sqrt(spring_gain / mass_);
	double delta_length = params.terminal_length - params.initial_length;
	double d_1 = initial_state.com_pos(rbd::Z) - params.initial_length + gravity_ /
			pow(spring_omega,2);
	double d_2 = initial_state.com_vel(rbd::Z) / spring_omega -
			delta_length / (spring_omega * params.duration);

	// Computing the preview trajectory
	unsigned int num_samples = round(params.duration / sample_time_);
	for (unsigned int k = 0; k < num_samples; k++) {
		double time = sample_time_ * k;

		// Computing the current time of the preview trajectory
		PreviewState current_state;
		current_state.time = initial_state.time + time;

		// Computing the horizontal motion of the CoM according to the SLIP system
		current_state.com_pos.head<2>() = beta_1 * exp(slip_omega * time) +
				beta_2 * exp(-slip_omega * time) +
				(cop_disp / params.duration) * time + params.initial_cop;
		current_state.com_vel.head<2>() = beta_1 * slip_omega * exp(slip_omega * time) -
				beta_2 * slip_omega * exp(-slip_omega * time) +
				cop_disp / params.duration;
		current_state.com_vel.head<2>() = beta_1 * pow(slip_omega,2) * exp(slip_omega * time) +
				beta_2 * pow(slip_omega,2) * exp(-slip_omega * time);

		// Computing the vertical motion of the CoM according to the spring-mass system
		current_state.com_pos(rbd::Z) = d_1 * cos(spring_omega * time) +
				d_2 * sin(spring_omega * time) + (delta_length / params.duration) * time +
				params.initial_length -	gravity_ / pow(spring_omega,2);
		current_state.com_vel(rbd::Z) = -d_1 * spring_omega * sin(spring_omega * time) +
				d_2 * spring_omega * cos(spring_omega * time) +
				delta_length / params.duration;
		current_state.com_acc(rbd::Z) = -d_1 * pow(spring_omega,2) * cos(spring_omega * time) -
				d_2 * pow(spring_omega,2) * sin(spring_omega * time);

		// Computing the heading motion according to heading kinematic equation
		current_state.head_pos = initial_state.head_pos + initial_state.head_vel * time +
				0.5 * params.head_acc * pow(time,2);
		current_state.head_vel = initial_state.head_vel + params.head_acc * time;
		current_state.head_acc = params.head_acc;

		// Appending the current state to the preview trajectory
		trajectory.push_back(current_state);
	}
}


void PreviewLocomotion::flightPreview(PreviewTrajectory& trajectory,
						   	   	   	  const PreviewState& initial_state,
									  const FlightPreviewParameters& params)
{
	// Setting the gravity vector
	Eigen::Vector3d gravity_vec = Eigen::Vector3d::Zero();
	gravity_vec(rbd::Z) = -gravity_;

	// Computing the preview trajectory
	unsigned int num_samples = round(params.duration / sample_time_);
	for (unsigned int k = 0; k < num_samples; k++) {
		double time = sample_time_ * k;

		// Computing the current time of the preview trajectory
		PreviewState current_state;
		current_state.time = initial_state.time + time;

		// Computing the CoM motion according to the projectile EoM
		current_state.com_pos = initial_state.com_pos + initial_state.com_vel * time +
				0.5 * gravity_vec * pow(time,2);
		current_state.com_vel = initial_state.com_vel + gravity_vec * time;
		current_state.com_acc = gravity_vec;

		// Computing the heading motion by assuming that there isn't change in the angular momentum
		current_state.head_pos = initial_state.head_pos + initial_state.head_vel * time;
		current_state.head_vel = initial_state.head_vel;
		current_state.head_acc = 0.;
	}
}

} //@namespace simulation
} //@namespace dwl
