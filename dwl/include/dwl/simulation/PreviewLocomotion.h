#ifndef DWL__SIMULATION__PREVIEW_LOCOMOTION__H
#define DWL__SIMULATION__PREVIEW_LOCOMOTION__H

#include <dwl/simulation/LinearControlledCartTableModel.h>
#include <dwl/simulation/FootSplinePatternGenerator.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/utils/DynamicLocomotion.h>
#include <dwl/utils/YamlWrapper.h>


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
	rbd::BodyPosition support_region;
	rbd::BodyVector foot_pos;
	rbd::BodyVector foot_vel;
	rbd::BodyVector foot_acc;
};

enum TypeOfPhases {STANCE, FLIGHT};
struct PreviewPhase
{
	PreviewPhase() : type(STANCE), feet(rbd::BodySelector()), step_(false) {}
	PreviewPhase(enum TypeOfPhases _type,
				 rbd::BodySelector _feet = rbd::BodySelector()) :
					 type(_type), feet(_feet), step_(false) {
		// Setting the swing feet of this phase
		for (unsigned int f = 0; f < feet.size(); f++)
			swing_feet[feet[f]] = true;

		// Checking if the phase makes a step
		if (feet.size() > 0)
			step_ = true;
	}

	void setTypeOfPhase(TypeOfPhases _type) {
		type = _type;
	}

	void setSwingFoot(std::string name) {
		swing_feet[name] = true;
	}

	bool isSwingFoot(std::string name) const {
		std::map<std::string,bool>::const_iterator it = swing_feet.find(name);
		if (it != swing_feet.end())
			return it->second;
		else
			return false;
	}

	bool doStep() const {
		return step_;
	}

	void setFootShift(std::string name, Eigen::Vector2d foot_shift) {
		feet_shift[name] = foot_shift;
	}

	TypeOfPhases getTypeOfPhase() {
		return type;
	}

	Eigen::Vector2d getFootShift(std::string name) const {
		rbd::BodyVector::const_iterator it = feet_shift.find(name);
		if (it != feet_shift.end())
			return it->second;
		else
			return Eigen::Vector2d::Zero();
	}

	TypeOfPhases type;
	rbd::BodySelector feet;
	std::map<std::string,bool> swing_feet;
	rbd::BodyVector feet_shift;
	bool step_;
};

struct PreviewParams
{
	PreviewParams() : duration(0.), cop_shift(Eigen::Vector2d::Zero()),
			head_acc(0.) {}
	PreviewParams(double _duration,
				  Eigen::Vector2d _cop_shift,
				  double _head_acc) : duration(_duration), cop_shift(_cop_shift),
						  head_acc(_head_acc) {}

	double duration;
	Eigen::Vector2d cop_shift;
	double head_acc;
	PreviewPhase phase;
};

struct PreviewControl
{
	PreviewControl() : params(std::vector<PreviewParams>()) {}
	PreviewControl(std::vector<PreviewParams> _params) : params(_params) {}

	std::vector<PreviewParams> params;
};

typedef std::vector<PreviewState> PreviewTrajectory;
typedef std::vector<PreviewPhase> PreviewSchedule;

struct SwingParams
{
	SwingParams() : duration(0.), feet_shift(rbd::BodyPosition()) {}
	SwingParams(double _duration,
				rbd::BodyPosition _feet_shift) : duration(_duration),
						feet_shift(_feet_shift) {}

	double duration;
	rbd::BodyPosition feet_shift;
};


/**
 * @class PreviewLocomotion
 * @brief Describes a preview locomotion
 * This class describes a preview of the locomotion by the decomposition of the
 * movements in two phases: stance and flight phase. In every phase, a preview
 * defines the dynamic response of the system by using a closed-loop equation.
 * These closed-loop equations describes the response of the locomotion
 * according to a predefined template model, i.e. low-dimensional model. For the
 * stance phase, we used a Spring Loaded Inverted Pendulum (SLIP) model to
 * describes the horizontal motion and spring-mass system for the vertical
 * motion of the CoM. Additionally, we assume the heading motion decouple from
 * the horizontal and vertical one. On the other hand, the flight phase is
 * modeled using the projectile EoM. Finally, the preview locomotion is a
 * consequence of a sequence of stance and flight previews.
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
		 * @param std::string Semantic system description filename
		 */
		void resetFromURDFFile(std::string urdf_file,
							   std::string system_file = std::string());

		/**
		 * @brief Resets the system information from URDF model
		 * @param std::string URDF model
		 * @param std::string Semantic system description filename
		 */
		void resetFromURDFModel(std::string urdf_model,
								std::string system_file = std::string());

		/**
		 * @brief Reads the preview sequence from a Yaml file
		 * @param PreviewState& Preview state
		 * @param PreviewControl& Preview control parameters
		 * @param std::string Filename
		 */
		void readPreviewSequence(PreviewState& state,
								 PreviewControl& control,
								 std::string filename);

		/**
		 * @brief Sets the sample time of the preview trajectory
		 * @param double Sample time
		 */
		void setSampleTime(double sample_time);

		/**
		 * @brief Sets the step height for the swing trajectory generation
		 * @param double Step height
		 */
		void setStepHeight(double step_height);

		/**
		 * @brief Sets the force threshold used for detecting active contacts
		 * @param double Force threshold
		 */
		void setForceThreshold(double force_threshold);

		void multiPhasePreview(PreviewTrajectory& trajectory,
							   const PreviewState& state,
							   const PreviewControl& control,
							   bool full = true);

		/**
		 * @brief Computes the total energy of the multi-phase preview
		 * @param Eigen::Vector3d& CoM energy
		 * @param const PreviewState& Initial state
		 * @param const PreviewControl& Preview schedule control
		 */
		void multiPhaseEnergy(Eigen::Vector3d& com_energy,
							  const PreviewState& state,
							  const PreviewControl& control);

		/**
		 * @brief Computes the preview of a stance phase
		 * The preview is computed according a Spring Loaded Linear
		 * Inverted Pendulum (SLIP) model, and by assuming that the
		 * Center of Pressure (CoP) and the pendulum length are linearly
		 * controlled
		 * @param PreviewTrajectory& Preview trajectory at the predefined
		 * sample time
		 * @param const PreviewState& Initial low-dimensional state
		 * @param const PreviewParams& Preview control parameters
		 * @param bool Label that indicates full preview or just the
		 * terminal state
		 */
		void stancePreview(PreviewTrajectory& trajectory,
						   const PreviewState& state,
						   const PreviewParams& params,
						   bool full = true);

		/**
		 * @brief Computes the preview of a flight phase
		 * The preview is computed according the projectile Equation
		 * of Motion (EoM), and assuming the non-changes in the
		 * angular momentum
		 * @param PreviewTrajectory& Preview trajectory at the predefined
		 * sample time
		 * @param const PreviewState& Initial low-dimensional state
		 * @param const PreviewParams& Preview control parameters
		 * @param bool Label that indicates full preview or just the
		 * terminal state
		 */
		void flightPreview(PreviewTrajectory& trajectory,
				   	   	   const PreviewState& state,
						   const PreviewParams& params,
						   bool full = true);

		/**
		 * @brief Computes the swing trajectory of the contact
		 * @param PreviewTrajectory& Preview trajectory at the predefined
		 * sample time
		 * @param const PreviewState& Initial low-dimensional state
		 * @param const SwingParams& Preview control parameters
		 */
		void addSwingPattern(PreviewTrajectory& trajectory,
							 const PreviewState& state,
							 const SwingParams& params);

		/** @brief Returns the floating-base system pointer */
		model::FloatingBaseSystem* getFloatingBaseSystem();

		/** @brief Returns the whole-body dynamics pointer */
		model::WholeBodyDynamics* getWholeBodyDynamics();

		/** @brief Returns the sample time */
		double getSampleTime();

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

		/**
		 * @brief Converts a preview trajectory to a whole-body trajectory
		 * @param WholeBodyTrajectory& Whole-body trajectory
		 * @param const PreviewTrajectory& Preview trajectory
		 */
		void toWholeBodyTrajectory(WholeBodyTrajectory& full_traj,
								   const PreviewTrajectory& preview_traj);


	private:
		/** @brief Feet spline generator */
		std::map<std::string,simulation::FootSplinePatternGenerator> feet_spline_generator_;

		/** @brief Floating-base system information */
		model::FloatingBaseSystem system_;

		/** @brief Whole-body dynamics */
		model::WholeBodyDynamics dynamics_;

		/** @brief Whole-body kinematics */
		model::WholeBodyKinematics kinematics_;

		/** @brief Label that indicates that the robot information from URDF
		 * was set */
		bool robot_model_;

		/** @brief Sample time of the preview trajectory */
		double sample_time_;

		/** @brief Gravity acceleration magnitude */
		double gravity_;

		/** @brief Total mass of the system */
		double mass_;

		/** @brief Number of feet */
		unsigned int num_feet_;

		/** @brief Feet names */
		rbd::BodySelector feet_names_;

		/** @brief Cart-table model */
		LinearControlledCartTableModel cart_table_;

		/** @brief Step height for the swing generation */
		double step_height_;

		/** @brief Actual Center of Mass (CoM) of the system */
		Eigen::Vector3d actual_system_com_;

		/** @ brief Stance posture position w.r.t. the CoM */
		rbd::BodyVector stance_posture_;

		/** @brief Force threshold */
		double force_threshold_;
};

} //@namespace simulation
} //@namespace dwl

#endif
