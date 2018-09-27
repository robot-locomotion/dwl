#ifndef DWL__SIMULATION__PREVIEW_LOCOMOTION__H
#define DWL__SIMULATION__PREVIEW_LOCOMOTION__H

#include <dwl/RobotStates.h>
#include <dwl/simulation/LinearControlledCartTableModel.h>
#include <dwl/simulation/FootSplinePatternGenerator.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/environment/TerrainMap.h>
#include <dwl/utils/YamlWrapper.h>
#include <dwl/utils/EigenExtra.h>


namespace dwl
{

namespace simulation
{

enum TypeOfPhases {STANCE, FLIGHT};
struct PreviewPhase
{
	PreviewPhase() : type(STANCE), feet(dwl::model::ElementList()), step_(false) {}
	PreviewPhase(const enum TypeOfPhases& _type,
				 const dwl::model::ElementList& _feet = dwl::model::ElementList()) :
					 type(_type), feet(_feet), step_(false) {
		// Setting the swing feet of this phase
		for (unsigned int f = 0; f < feet.size(); ++f)
			swing_feet[feet[f]] = true;

		// Checking if the phase makes a step
		if (feet.size() > 0)
			step_ = true;
	}

	void setTypeOfPhase(const TypeOfPhases& _type) {
		type = _type;
	}

	void setSwingFoot(const std::string& name) {
		swing_feet[name] = true;
	}

	bool isSwingFoot(const std::string& name) const {
		std::map<std::string,bool>::const_iterator it = swing_feet.find(name);
		if (it != swing_feet.end())
			return it->second;
		else
			return false;
	}

	bool doStep() const {
		return step_;
	}

	void setFootShift(const std::string& name,
					  const Eigen::Vector2d& foot_shift) {
		feet_shift[name] = foot_shift;
	}

	TypeOfPhases getTypeOfPhase() {
		return type;
	}

	Eigen::Vector2d getFootShift(const std::string& name) const {
		Eigen::Vector2dMap::const_iterator it = feet_shift.find(name);
		if (it != feet_shift.end())
			return it->second;
		else
			return Eigen::Vector2d::Zero();
	}

	TypeOfPhases type;
	dwl::model::ElementList feet;
	std::map<std::string,bool> swing_feet;
	Eigen::Vector2dMap feet_shift;
	bool step_;
};

struct PreviewParams
{
	PreviewParams() : id(0), duration(0.), cop_shift(Eigen::Vector2d::Zero()) {}
	PreviewParams(const double& _duration,
				  const Eigen::Vector2d& _cop_shift) : id(0),
						  duration(_duration),
						  cop_shift(_cop_shift) {}
	PreviewParams(const unsigned int _id,
				  const double& _duration,
				  const Eigen::Vector2d& _cop_shift) : id(_id),
							  duration(_duration),
							  cop_shift(_cop_shift) {}

	unsigned int id;
	double duration;
	Eigen::Vector2d cop_shift;
	PreviewPhase phase;
};

struct PreviewControl
{
	PreviewControl() : params(std::vector<PreviewParams>()) {}
	PreviewControl(const std::vector<PreviewParams>& _params) : params(_params) {}

	double getTotalDuration() const {
		double duration = 0.;
		for (unsigned int i = 0; i < params.size(); ++i)
			duration += params[i].duration;

		return duration;
	}

	std::vector<PreviewParams> params;
};

struct PreviewSchedule
{
	PreviewSchedule() : actual_phase_(0), next_phase_(0) {}

	void init(const unsigned int& initial_phase) {
		actual_phase_ = initial_phase;
	}

	void init(const SE3Map& support) {
		for (unsigned int p = 0; p < getNumberPhases(); ++p) {
			std::vector<std::string> swings = getSwingFeet(p);
			unsigned int num_swings = getNumberOfSwingFeet(p);

			bool is_phase = true;
			if (num_swings == 0) {
				if (support.size() == feet.size()) {
					actual_phase_ = p;
				}
			} else {
				for (unsigned int s = 0; s < num_swings; ++s) {
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

	void addPhase(const PreviewPhase& phase) {
		schedule.push_back(phase);
	}

	void setFeet(const dwl::model::ElementList& _feet) {
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

	PreviewPhase& getPhase(const unsigned int& index) {
		return schedule[index];
	}

	TypeOfPhases getTypeOfPhase(const unsigned int& index) {
		return schedule[index].type;
	}

	unsigned int getNumberOfSwingFeet(const unsigned int& index) {
		return schedule[index].feet.size();
	}

	std::vector<std::string>& getSwingFeet(const unsigned int& index) {
		return schedule[index].feet;
	}

	unsigned int getNumberPhases() {
		return schedule.size();
	}

	std::vector<PreviewPhase> schedule;
	dwl::model::ElementList feet;
	unsigned int actual_phase_;
	unsigned int next_phase_;
};


struct SwingParams
{
	SwingParams() : duration(0.), feet_shift(Eigen::Vector3dMap()) {}
	SwingParams(const double& _duration,
				const Eigen::Vector3dMap& _feet_shift) : duration(_duration),
						feet_shift(_feet_shift) {}

	double duration;
	Eigen::Vector3dMap feet_shift;
};


struct VelocityCommand
{
	VelocityCommand() : linear(Eigen::Vector2d::Zero()), angular(0.) {}
	VelocityCommand(const Eigen::Vector2d& _linear,
					const double& _angular) : linear(_linear), angular(_angular) {}

	Eigen::Vector2d linear;
	double angular;
};

typedef std::map<std::string,bool> Support;
typedef Support::const_iterator SupportIterator;

struct PreviewState
{
	PreviewState() : height(0.), support(Support()) {
		com_pos.setZero();
		com_vel.setZero();
	}

	PreviewState(const double& _height,
				 const Eigen::Vector2d& _com_pos,
				 const Eigen::Vector2d& _com_vel,
				 const Support& _support) : height(_height),
						 com_pos(_com_pos), com_vel(_com_vel),
						 support(_support) {}

	PreviewState(const ReducedBodyState& state) {
		// Frame transformer
		math::FrameTF tf;

		Eigen::Vector3d com_disp_W =
				state.getCoMSE3().getTranslation() - state.getCoPPosition_W();
		Eigen::Vector3d com_disp_H =
				tf.fromWorldToHorizontalFrame(com_disp_W, state.com_pos.getRPY());

		height = com_disp_H(rbd::Z);
		com_pos = (Eigen::Vector2d) com_disp_H.head<2>();
		com_vel = (Eigen::Vector2d) state.getCoMVelocity_H().getLinear().head<2>();
		for (SE3Map::const_iterator it = state.support_region.begin();
				it != state.support_region.end(); ++it)
			support[it->first] = true;
	}

	double height;
	Eigen::Vector2d com_pos;
	Eigen::Vector2d com_vel;
	Support support;
};


struct PreviewSets
{
	PreviewSets() : command(VelocityCommand()),
			state(PreviewState()), control(PreviewControl()){};
	PreviewSets(const VelocityCommand& _command,
				const PreviewState& _state,
				const PreviewControl& _control) :
					command(_command), state(_state), control(_control) {}

	VelocityCommand command;
	PreviewState state;
	PreviewControl control;
};

typedef std::vector<PreviewSets> PreviewData;

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

		/** @brief Resets the robot dynamics
		 * @param[in] fbs Floating-base model
		 * @param[in] wdyn Whole-body dynamics
		 **/
		void reset(model::FloatingBaseSystem& fbs,
				   model::WholeBodyKinematics& wkin,
				   model::WholeBodyDynamics& wdyn);

		/**
		 * @brief Reads the entire preview sequence from a Yaml file
		 * @param PreviewData& Preview data
		 * @param std::string Filename
		 */
		void readPreviewSequence(PreviewData& data,
								 std::string filename);

		/**
		 * @brief Reads the preview sequence from a Yaml file defined in
		 * specific namespace
		 * @param VelocityCommand& Velocity command
		 * @param PreviewState& Preview state
		 * @param PreviewControl& Preview control parameters
		 * @param std::string Filename
		 * @param YamlNamespace Yaml namspace where is defined the preview sequence
		 */
		void readPreviewSequence(VelocityCommand& command,
								 PreviewState& state,
								 PreviewControl& control,
								 std::string filename,
								 YamlNamespace seq_ns);

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
		std::shared_ptr<model::FloatingBaseSystem> getFloatingBaseSystem();

		/** @brief Returns the whole-body dynamics pointer */
		std::shared_ptr<model::WholeBodyDynamics> getWholeBodyDynamics();

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

		/** @brief Floating-base system */
		std::shared_ptr<model::FloatingBaseSystem> fbs_;

		/** @brief Whole-body kinematics */
		std::shared_ptr<model::WholeBodyKinematics> wkin_;

		/** @brief Whole-body dynamics */
		std::shared_ptr<model::WholeBodyDynamics> wdyn_;

		/** @brief Robot state converter */
		RobotStates state_tf_;

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
		dwl::model::ElementList feet_names_;

		/** @brief Cart-table model */
		LinearControlledCartTableModel cart_table_;

		/** @brief Step height for the swing generation */
		double step_height_;

		/** @ brief Stance posture position w.r.t. the horizontal frame */
		SE3Map stance_posture_H_;
};

} //@namespace simulation
} //@namespace dwl

#endif
