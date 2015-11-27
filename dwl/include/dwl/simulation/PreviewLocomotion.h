#ifndef DWL__SIMULATION__PREVIEW_LOCOMOTION__H
#define DWL__SIMULATION__PREVIEW_LOCOMOTION__H

#include <dwl/model/FloatingBaseSystem.h>


namespace dwl
{

namespace simulation
{

struct PreviewParameters
{
	double duration;
	Eigen::Vector2d initial_cop;
	Eigen::Vector2d terminal_cop;
	double initial_lenght;
	double terminal_lenght;
	double heading_acc;
};

/**
 * @class PreviewLocomotion
 * @brief Describes a preview locomotion
 * This class describes a preview of the locomotion by the decomposition of the movements in two
 * phases: stance and flight phase. In every phase, a preview defines the dynamic response of the
 * system by using a closed-loop equation. These closed-loop equations describes the response of
 * the locomotion according to a predefined template model, i.e. low-dimensional model. For the
 * stance phase, we used a Spring Loaded Inverted Pendulum (SLIP) model to describes the horizontal
 * motion and spring-mass system for the vertical motion of the CoM. Additionally, we assume the
 * heading motion decouple from the horizontal and vertical one. On the other hand, the flight phase
 * is modeled using the projectile EoM. Finally, the preview locomotion is a consequence of a
 * sequence of stance and flight previews.
 */
class PreviewLocomotion
{
	public:
		/** @brief Constructor function */
		PreviewLocomotion();

		/** @brief Destructor function */
		~PreviewLocomotion();

		/**
		 * @brief Resets the system information from an URDF file
		 * @param std::string URDF filename
		 */
		void resetFromURDFFile(std::string filename);

		void setSampleTime(double sample_time);

		/**
		 * @brief Resets the system information from URDF model
		 * @param std::string URDF model
		 */
		void resetFromURDFModel(std::string urdf_model);

		void previewScheduled(WholeBodyTrajectory& trajectory,
							  std::vector<PreviewParameters> control_params);

		void stancePreview(WholeBodyState& state, double time);
		void flightPreview(WholeBodyState& state, double time);

	private:
		void evalutateSLIP(double com, double time);

		model::FloatingBaseSystem system_;
		double sample_time_;
};

} //@namespace simulation
} //@namespace dwl

#endif
