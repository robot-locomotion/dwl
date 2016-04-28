#ifndef DWL__ROBOT__ROBOT__H
#define DWL__ROBOT__ROBOT__H

#include <dwl/behavior/MotorPrimitives.h>
#include <dwl/utils/utils.h>
#include <dwl/utils/YamlWrapper.h>


namespace dwl
{

namespace robot
{

//enum QuadrupeLegID {LF, RF, LH, RH};
//enum HumanoidLegID {L, R};
//enum EndEffectorID {LF_foot, RF_foot, LH_foot, RH_foot};


/**
 * @class Robot
 * @brief Class for defining the properties of the robot
 */
class Robot
{
	public:
		/** @brief Constructor function */
		Robot();

		/** @brief Destructor function */
		~Robot();

		/**
		 * @brief Reads the robot properties from a yaml file
		 * @param std::string File name of the yaml
		 */
		void read(std::string filename);

		/**
		 * @brief Sets the current pose of the robot
		 * @param const Pose& Current pose
		 */
		void setCurrentPose(const Pose& pose);

		/**
		 * @brief Sets the current contacts of the robot
		 * @param const std::vector<Contact>& Contact positions
		 */
		void setCurrentContacts(const std::vector<Contact>& contacts);

		/**
		 * @brief Gets current pose of the robot
		 * @return The current pose of the robot
		 */
		Pose getCurrentPose();

		/**
		 * @brief Gets the current contact positions
		 * @return The set of current contacts
		 */
		std::vector<Contact> getCurrentContacts();

		/**
		 * @brief Gets the body motor primitives
		 * @return The body motor primitives
		 */
		behavior::MotorPrimitives& getBodyMotorPrimitive();

		/**
		 * @brief Gets the predefined body workspace for evaluation of
		 * potential collisions
		 * @return The predefined body workspace
		 */
		SearchArea getPredefinedBodyWorkspace();

		/**
		 * @brief Gets the predefined leg workspace for evaluation of
		 * potential collisions
		 * @return The predefined leg workspaces
		 */
		SearchAreaMap getPredefinedLegWorkspaces();

		/**
		 * @brief Gets the current stance of the robot
		 * @param const Eigen::Vector3d& action Action to execute
		 * @return The current stance of the robot
		 */
		Vector3dMap getStance(const Eigen::Vector3d& action = Eigen::Vector3d::Zero());

		/**
		 * @brief Gets the nominal stance of the robot
		 * @return The nominal stance of the robot
		 */
		Vector3dMap getNominalStance();

		/**
		 * @brief Gets the pattern of locomotion of the robot
		 * @return The pattern of locomotion
		 */
		PatternOfLocomotionMap getPatternOfLocomotion();

		/**
		 * @brief Gets the stance areas
		 * @param const Eigen::Vector3d& action Action to execute
		 * @return The stance areas
		 */
		SearchAreaMap getFootstepSearchAreas(const Eigen::Vector3d& action = Eigen::Vector3d::Zero());

		/**
		 * @brief Gets the footstep search region given an action
		 * @param const Eigen::Vector3d& action Action to execute
		 * @return The footstep search regions
		 */
		SearchAreaMap getFootstepSearchSize(const Eigen::Vector3d& action = Eigen::Vector3d::Zero());

		/**
		 * @brief Gets the expected ground according to the nominal stance
		 * @param int Foot id
		 * @return The expected ground according to the nominal stance of the leg
		 */
		double getExpectedGround(int foot_id);

		/** @brief Gets the number of legs of the robot */
		double getNumberOfLegs();

		/** @brief Gets the end-effector map */
		EndEffectorMap getEndEffectorMap();

		/** @brief Gets the leg map */
		EndEffectorMap getLegMap();


	protected:
		/** @brief Current pose of the robot */
		Pose current_pose_;

		/** @brief Current contacts of the robot */
		std::vector<Contact> current_contacts_;

		/** @brief Pointer to the body motor primitives */
		behavior::MotorPrimitives* body_behavior_;

		/** @brief Yaml bridge */
		YamlWrapper yaml_reader_;

		/** @brief End-Effector map */
		EndEffectorMap end_effectors_;

		/** @brief Feet map */
		EndEffectorMap feet_;

		/** @brief Patch map */
		PatchMap patchs_;//TODO evaluate if it's required

		/** @brief Vector of the nominal stance */
		Vector3dMap nominal_stance_;

		/** @brief Foot workspaces */
		SearchAreaMap foot_workspaces_;

		/** @brief Footstep window */
		SearchAreaMap footstep_window_;

		/** @brief Vector of the body workspace */
		SearchArea body_workspace_;

		/** @brief Vector of footstep search areas */
		std::vector<SearchArea> footstep_search_areas_;

		/** @brief Pattern of locomotion */
		PatternOfLocomotionMap pattern_locomotion_;

		/** @brief Number of feet */
		double num_feet_;

		/** @brief Number of end-effectors */
		double num_end_effectors_;

		/** @brief Estimated ground from the body frame */
		double estimated_ground_from_body_;

		/** @brief The last past foot */
		int last_past_foot_;

		/** @brief Lateral offset between the feet */
		double feet_lateral_offset_;

		/** @brief Body displacement */
		double displacement_;
};

} //@namespace robot
} //@namespace dwl

#endif
