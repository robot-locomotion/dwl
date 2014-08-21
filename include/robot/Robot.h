#ifndef DWL_Robot_H
#define DWL_Robot_H

#include <utils/utils.h>


namespace dwl
{

namespace robot
{

enum QuadrupeLegID {LF, RF, LH, RH};
enum HumanoidLegID {L, R};

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
		 * @brief Sets the current pose of the robot
		 * @param dwl::Pose pose Current pose
		 */
		void setCurrentPose(Pose pose);

		/**
		 * @brief Sets the stance areas for computing the body adjacency map
		 * @param std::vector<SearchArea> stance_areas Stance areas
		 */
		void setStanceAreas(std::vector<SearchArea> stance_areas); //TODO Re-write this method for general settings

		/**
		 * @brief Sets the pattern of locomotion of the robot
		 * @param std::vector<int> pattern Sequence of leg movements according to the current leg
		 */
		void setPatternOfLocomotion(std::vector<int> pattern);

		/**
		 * @brief Gets current pose of the robot
		 * @return dwl::Pose Returns the current pose of the robot
		 */
		Pose getCurrentPose();

		/**
		 * @brief Gets the body area
		 * @return dwl::Area Return the body area of the robot
		 */
		Area getBodyArea();

		/**
		 * @brief Gets the nominal stance of the robot
		 * @return std::vector<Eigen::Vector3d> Returns the nominal stance of the robot
		 */
		std::vector<Eigen::Vector3d> getNominalStance();

		/**
		 * @brief Gets the pattern of locomotion of the robot
		 * @return std::vector<int> Returns the pattern of locomotion
		 */
		std::vector<int> getPatternOfLocomotion();

		/**
		 * @brief Gets the stance areas
		 * @return std::vector<SearchArea> Returns the stance areas
		 */
		std::vector<SearchArea> getStanceAreas();

		/**
		 * @brief Gets the expected ground according to the nominal stance
		 * @param int leg_id Leg id
		 * @return std::vector<double> Returns the expected ground according to the nominal stance of the leg
		 */
		double getExpectedGround(int leg_id);

		/**
		 * @brief Gets the leg work-areas for evaluation of potential collisions
		 * @return std::vector<SearchArea> Returns the leg work-areas
		 */
		std::vector<SearchArea> getLegWorkAreas();

		/** @brief Gets the number of legs of the robot */
		double getNumberOfLegs();


	protected:
		/** @brief Current pose of the robot */
		Pose current_pose_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		/** @brief Vector of the body area */
		Area body_area_;

		/** @brief Vector of the nominal stance */
		std::vector<Eigen::Vector3d> nominal_stance_;

		/** @brief Pattern of locomotion */
		std::vector<int> pattern_locomotion_;

		/** @brief Number of legs */
		double number_legs_;

		double stance_size_;

		std::vector<SearchArea> leg_area_;
};

} //@namespace robot
} //@namespace dwl

#endif
