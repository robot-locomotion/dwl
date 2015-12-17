#ifndef DWL__SIMULATION__PREVIEW_LOCOMOTION__H
#define DWL__SIMULATION__PREVIEW_LOCOMOTION__H

#include <dwl/simulation/FootSplinePatternGenerator.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/utils/DynamicLocomotion.h>


namespace dwl
{

namespace simulation
{

struct PreviewState
{
	PreviewState() : time(0.), head_pos(0.), head_vel(0.), head_acc(0.) {
		com_pos.setZero();
		com_vel.setZero();
		com_acc.setZero();
		cop.setZero();
	}

	double time;
	Eigen::Vector3d com_pos;
	Eigen::Vector3d com_vel;
	Eigen::Vector3d com_acc;
	double head_pos;
	double head_vel;
	double head_acc;
	Eigen::Vector3d cop;
	rbd::BodyVector support_region;
};

typedef std::vector<PreviewState> PreviewTrajectory;

struct StancePreviewParameters
{
	double duration;
	Eigen::Vector2d terminal_cop;
	double terminal_length;
	double head_acc;
};

struct FlightPreviewParameters
{
	double duration;
};

struct QuadrupedalPreviewParameters
{
	StancePreviewParameters four_support;
	StancePreviewParameters three_support;
	StancePreviewParameters two_support;
	FlightPreviewParameters flight;
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

		/**
		 * @brief Resets the system information from URDF model
		 * @param std::string URDF model
		 */
		void resetFromURDFModel(std::string urdf_model);

		/**
		 * @brief Sets the sample time of the preview trajectory
		 * @param double Sample time
		 */
		void setSampleTime(double sample_time);

		/**
		 * @brief Sets the spring gain that module the vertical movement
		 * @param double gain Spring gain
		 */
		void setSpringGain(double gain);


		void previewScheduled(PreviewTrajectory& trajectory,
							  const PreviewState& state,
							  const std::vector<QuadrupedalPreviewParameters>& control_params);

		/**
		 * @brief Computes the preview of the stance-phase given the stance parameters
		 * The preview is computed according a Spring Linear Inverted Pendulum (SLIP) model, and by
		 * assuming that the Center of Pressure (CoP) and the pendulum length are linearly controlled
		 * @param PreviewTrajectory& Preview trajectory at the predefined sample time
		 * @param const PreviewState& Initial low-dimensional state
		 * @param const StancePreviewParameters& Preview parameters
		 */
		void stancePreview(PreviewTrajectory& trajectory,
						   const PreviewState& state,
						   const StancePreviewParameters& params);

		/**
		 * @brief Computes the preview of the flight-phase given the duration of the phase
		 * The preview is computed according the projectile Equation of Motion (EoM), and assuming
		 * the non-changes in the angular momentum
		 * @param PreviewTrajectory& Preview trajectory at the predefined sample time
		 * @param const PreviewState& Initial low-dimensional state
		 * @param const FlightPreviewParameters& Preview parameters
		 */
		void flightPreview(PreviewTrajectory& trajectory,
				   	   	   const PreviewState& state,
						   const FlightPreviewParameters& params);

		/**
		 * @brief Converts the preview state vector to whole-body state
		 * @param WholeBodyState& Whole-body state
		 * @param const PreviewStated& Preview state vector
		 */
		void toWholeBodyState(WholeBodyState& full_state,
							  const PreviewState& preview_state);

		/**
		 * @brief Converts the whole-body state to preview state vector
		 * @param PreviewStated& Preview state vector
		 * @param const WholeBodyState& Whole-body state
		 */
		void fromWholeBodyState(PreviewState& preview_state,
								const WholeBodyState& full_state);



	private:
		/** @brief Foot pattern generator */
		simulation::FootSplinePatternGenerator foot_pattern_generator_;

		/** @brief Floating-base system information */
		model::FloatingBaseSystem system_;

		/** @brief Whole-body dynamics */
		model::WholeBodyDynamics dynamics_;

		/** @brief Sample time of the preview trajectory */
		double sample_time_;

		/** @brief Gravity acceleration magnitude */
		double gravity_;

		/** @brief Total mass of the system */
		double mass_;

		/** @brief Spring gain that module the vertical movement */
		double spring_gain_;

		/** @brief Base Center of Mass (CoM) */
		Eigen::Vector3d base_com_;
};

} //@namespace simulation
} //@namespace dwl

#endif
