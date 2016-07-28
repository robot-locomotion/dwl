#ifndef DWL__SIMULATION__PREVIEW_LOCOMOTION__H
#define DWL__SIMULATION__PREVIEW_LOCOMOTION__H

#include <dwl/simulation/LinearControlledCartTableModel.h>
#include <dwl/simulation/FootSplinePatternGenerator.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/environment/TerrainMap.h>
#include <dwl/utils/YamlWrapper.h>


namespace dwl
{

namespace simulation
{

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

struct PreviewSchedule
{
	PreviewSchedule() : actual_phase_(0), next_phase_(0) {}

	void init(unsigned int initial_phase) {
		actual_phase_ = initial_phase;
	}

	void init(rbd::BodyPosition& support) {
		for (unsigned int p = 0; p < getNumberPhases(); p++) {
			std::vector<std::string> swings = getSwingFeet(p);
			unsigned int num_swings = getNumberOfSwingFeet(p);

			bool is_phase = true;
			if (num_swings == 0) {
				if (support.size() == feet.size()) {
					actual_phase_ = p;
				}
			} else {
				for (unsigned int s = 0; s < num_swings; s++) {
					std::string swing = swings[s];

					if (support.find(swing) != support.end()) {
						is_phase = false;
						break;
					}
				}

				if (is_phase) {
					actual_phase_ = p;
				}
			}
		}
	}

	void addPhase(PreviewPhase phase) {
		schedule.push_back(phase);
	}

	void setFeet(rbd::BodySelector& _feet) {
		feet = _feet;
	}

	unsigned int getIndex() {
		// Updating the actual phase
		actual_phase_ = next_phase_;

		// Computing the next phase
		++next_phase_;
		if (next_phase_ == getNumberPhases())
			next_phase_ = 0;

		return actual_phase_;
	}

	PreviewPhase& getPhase(unsigned int index) {
		return schedule[index];
	}

	TypeOfPhases getTypeOfPhase(unsigned int index) {
		return schedule[index].type;
	}

	unsigned int getNumberOfSwingFeet(unsigned int index) {
		return schedule[index].feet.size();
	}

	std::vector<std::string>& getSwingFeet(unsigned int index) {
		return schedule[index].feet;
	}

	unsigned int getNumberPhases() {
		return schedule.size();
	}

	std::vector<PreviewPhase> schedule;
	rbd::BodySelector feet;
	unsigned int actual_phase_;
	unsigned int next_phase_;
};


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
		 * @param ReducedBodyState& Reduced-body state
		 * @param PreviewControl& Preview control parameters
		 * @param std::string Filename
		 */
		void readPreviewSequence(ReducedBodyState& state,
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

		/**
		 * @brief Computes the multi-phase preview trajectory
		 * @param ReducedBodyTrajectory& Reduced-body trajectory
		 * @param const ReducedBodyState& Actual reduced-body state
		 * @param const PreviewControl& Preview control
		 * @param bool True for the full trajectory, otherwise compute the
		 * preview state transitions
		 */
		void multiPhasePreview(ReducedBodyTrajectory& trajectory,
							   const ReducedBodyState& state,
							   const PreviewControl& control,
							   bool full = true);

		/**
		 * @brief Computes the total energy of the multi-phase preview
		 * @param Eigen::Vector3d& CoM energy
		 * @param const ReducedBodyState& Initial state
		 * @param const PreviewControl& Preview schedule control
		 */
		void multiPhaseEnergy(Eigen::Vector3d& com_energy,
							  const ReducedBodyState& state,
							  const PreviewControl& control);

		/**
		 * @brief Computes the preview of a stance phase
		 * The preview is computed according a Spring Loaded Linear
		 * Inverted Pendulum (SLIP) model, and by assuming that the
		 * Center of Pressure (CoP) and the pendulum length are linearly
		 * controlled
		 * @param ReducedTrajectory& Reduced-body trajectory at the predefined
		 * sample time
		 * @param const ReducedBodyState& Initial low-dimensional state
		 * @param const PreviewParams& Preview control parameters
		 * @param bool Label that indicates full preview or just the
		 * terminal state
		 */
		void stancePreview(ReducedBodyTrajectory& trajectory,
						   const ReducedBodyState& state,
						   const PreviewParams& params,
						   bool full = true);

		/**
		 * @brief Computes the preview of a flight phase
		 * The preview is computed according the projectile Equation
		 * of Motion (EoM), and assuming the non-changes in the
		 * angular momentum
		 * @param ReducedBodyTrajectory& Reduced-body trajectory at the predefined
		 * sample time
		 * @param const ReducedBodyState& Initial low-dimensional state
		 * @param const PreviewParams& Preview control parameters
		 * @param bool Label that indicates full preview or just the
		 * terminal state
		 */
		void flightPreview(ReducedBodyTrajectory& trajectory,
				   	   	   const ReducedBodyState& state,
						   const PreviewParams& params,
						   bool full = true);

		/**
		 * @brief Initializes the swing trajectory generator
		 * @param const ReducedBodyState& Initial low-dimensional state
		 * @param const PreviewParams& Preview control parameters
		 */
		void initSwing(const ReducedBodyState& state,
					   const PreviewParams& params);

		/**
		 * @brief Generates the swing trajectory of the contact
		 * @param ReducedBodyState& Desired preview state
		 * sample time
		 */
		void generateSwing(ReducedBodyState& state,
						   double time);

		/** @brief Returns the floating-base system pointer */
		model::FloatingBaseSystem* getFloatingBaseSystem();

		/** @brief Returns the whole-body dynamics pointer */
		model::WholeBodyDynamics* getWholeBodyDynamics();

		/** @brief Returns the terrain map pointer */
		environment::TerrainMap* getTerrainMap();

		/** @brief Returns the sample time */
		double getSampleTime();

		/**
		 * @brief Converts the reduced-body state to whole-body one
		 * @param WholeBodyState& Whole-body state
		 * @param const ReducedBodyStated& Reduced-body state
		 */
		void toWholeBodyState(WholeBodyState& full_state,
							  const ReducedBodyState& reduced_state);

		/**
		 * @brief Converts the whole-body state to reduced-body state one
		 * @param ReducedBodyStated& Reduced-body state
		 * @param const WholeBodyState& Whole-body state
		 */
		void fromWholeBodyState(ReducedBodyState& reduced_state,
								const WholeBodyState& full_state);

		/**
		 * @brief Converts a reduced-body trajectory to a whole-body one
		 * @param WholeBodyTrajectory& Whole-body trajectory
		 * @param const ReducedBodyTrajectory& Reduced-body trajectory
		 */
		void toWholeBodyTrajectory(WholeBodyTrajectory& full_traj,
								   const ReducedBodyTrajectory& reduced_traj);


	private:
		/** @brief Actual reduced-body state */
		ReducedBodyState actual_state_;

		/** @brief Feet spline generator */
		FootSplinerMap feet_spline_generator_;
		SwingParams swing_params_;
		ReducedBodyState phase_state_;

		/** @brief Floating-base system information */
		model::FloatingBaseSystem system_;

		/** @brief Whole-body dynamics */
		model::WholeBodyDynamics dynamics_;

		/** @brief Whole-body kinematics */
		model::WholeBodyKinematics kinematics_;

		/** @brief Terrain map */
		environment::TerrainMap terrain_;

		/** @brief Frame transformations */
		math::FrameTF frame_tf_;

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

		/** @brief Center of Mass (CoM) position of the system w.r.t. the base
		 * frame */
		Eigen::Vector3d com_pos_B_;

		/** @ brief Stance posture position w.r.t. the CoM */
		rbd::BodyVector stance_posture_;

		/** @brief Force threshold */
		double force_threshold_;
};

} //@namespace simulation
} //@namespace dwl

#endif
