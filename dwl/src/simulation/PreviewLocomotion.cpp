#include <dwl/simulation/PreviewLocomotion.h>


namespace dwl
{

namespace simulation
{

PreviewLocomotion::PreviewLocomotion() : sample_time_(0.001)
{

}


PreviewLocomotion::~PreviewLocomotion()
{

}


void PreviewLocomotion::resetFromURDFFile(std::string filename)
{
	system_.resetFromURDFFile(filename);
}


void PreviewLocomotion::resetFromURDFModel(std::string urdf_model)
{
	system_.resetFromURDFModel(urdf_model);
}


void PreviewLocomotion::setSampleTime(double sample_time)
{
	sample_time_ = sample_time;
}


void PreviewLocomotion::previewScheduled(WholeBodyTrajectory& trajectory,
										 std::vector<PreviewParameters> control_params)
{
	// Computing the number of samples
	double preview_duration = 0.;
	unsigned int num_phases = control_params.size();
	for (unsigned int i = 0; i < num_phases; i++)
		preview_duration += control_params[i].duration;
	unsigned int num_samples = round(preview_duration / sample_time_);

	// Computing the preview locomotion trajectory
	for (unsigned int k = 0; k < num_samples; k++) {
		double current_time = sample_time_ * k;
	}
}


void PreviewLocomotion::stancePreview(WholeBodyState& state, double time)
{
//	WholeBodyState initial_state, terminal_state;

	double stance_time;
	double com0_pos, com0_vel;
	double cop0_pos;
	double delta_cop;
	double gravity = 9.81;
	double pendulum_height;
	double natural_freq = sqrt(gravity / pendulum_height);
	double alpha = 2 * natural_freq * stance_time;
	double beta_1 = (com0_pos - cop0_pos) / 2 + (com0_vel * stance_time - delta_cop) / alpha;
	double beta_2 = (com0_pos - cop0_pos) / 2 - (com0_vel * stance_time - delta_cop) / alpha;

	double x = beta_1 * exp(natural_freq * time) + beta_2 * exp(-natural_freq * time) +
			(delta_cop / stance_time) * time + cop0_pos;
}


void PreviewLocomotion::evalutateSLIP(double com, double time)
{

}

} //@namespace simulation
} //@namespace dwl
