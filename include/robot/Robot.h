#ifndef DWL_Robot_H
#define DWL_Robot_H

#include <behavior/MotorPrimitives.h>
#include <utils/utils.h>
#include <utils/YamlBridge.h>


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
		 * @param std::string filepath File path of the yaml
		 */
		void read(std::string filepath);

		/**
		 * @brief Sets the current pose of the robot
		 * @param dwl::Pose pose Current pose
		 */
		void setCurrentPose(Pose pose);

		/**
		 * @brief Sets the current contacts of the robot
		 * @param std::vector<dwl::Contact> contacts Contact positions
		 */
		void setCurrentContacts(std::vector<Contact> contacts);

		/**
		 * @brief Gets current pose of the robot
		 * @return dwl::Pose Returns the current pose of the robot
		 */
		Pose getCurrentPose();

		/**
		 * @brief Gets the current contact positions
		 * @return std::vector<Contact> Returns the set of current contacts
		 */
		std::vector<Contact> getCurrentContacts();

		/**
		 * @brief Gets the body motor primitives
		 * @return behavior::MotorPrimitives& Returns the body motor primitives
		 */
		behavior::MotorPrimitives& getBodyMotorPrimitive();

		/**
		 * @brief Gets the predefined body workspace for evaluation of potential collisions
		 * @return dwl::SearchArea Returns the predefined body workspace
		 */
		SearchArea getPredefinedBodyWorkspace();

		/**
		 * @brief Gets the predefined leg workspace for evaluation of potential collisions
		 * @return SearchAreaMap Returns the predefined leg workspaces
		 */
		SearchAreaMap getPredefinedLegWorkspaces();

		/**
		 * @brief Gets the current stance of the robot
		 * @param Eigen::Vector3d action Action to execute
		 * @return dwl::Vector3dMap Returns the current stance of the robot
		 */
		Vector3dMap getStance(Eigen::Vector3d action);

		/**
		 * @brief Gets the pattern of locomotion of the robot
		 * @return dwl::PatternOfLocomotionMap Returns the pattern of locomotion
		 */
		PatternOfLocomotionMap getPatternOfLocomotion();

		/**
		 * @brief Gets the stance areas
		 * @param Eigen::Vector3d action Action to execute
		 * @return dwl::SearchAreaMap Returns the stance areas
		 */
		SearchAreaMap getFootstepSearchAreas(Eigen::Vector3d action);

		/**
		 * @brief Gets the footstep search region given an action
		 * @param Eigen::Vector3d action Action to execute
		 * @return dwl::SearchAreaMap Returns the footstep search regions
		 */
		SearchAreaMap getFootstepSearchSize(Eigen::Vector3d action);

		/**
		 * @brief Gets the expected ground according to the nominal stance
		 * @param int leg_id Leg id
		 * @return std::vector<double> Returns the expected ground according to the nominal stance of the leg
		 */
		double getExpectedGround(int leg_id);

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
		YamlBridge yaml_reader_;

		/** @brief End-Effector map */
		EndEffectorMap end_effectors_;

		// TODO
		EndEffectorMap feet_;
		PatchMap patchs_;

		/** @brief Vector of the nominal stance */
		Vector3dMap nominal_stance_;

		/** @brief Leg workspaces */
		SearchAreaMap leg_workspaces_;

		/** @brief Footstep window */
		SearchAreaMap footstep_window_;

		/** @brief Vector of the body workspace */
		SearchArea body_workspace_;

		/** @brief Vector of footstep search areas */
		std::vector<SearchArea> footstep_search_areas_;

		/** @brief Pattern of locomotion */
		PatternOfLocomotionMap pattern_locomotion_;

		/** @brief Number of legs */
		double number_legs_;

		/** @brief Number of end-effectors */
		double number_end_effectors_;

		/** @brief Estimated ground from the body frame */
		double estimated_ground_from_body_;

		/** @brief The last past leg */
		int last_past_leg_;

		/** @brief Lateral offset between the legs */
		double leg_lateral_offset_;

		/** @brief Body displacement */
		double displacement_;
};

} //@namespace robot
} //@namespace dwl

#endif
