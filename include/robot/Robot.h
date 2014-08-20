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

		void setPose(Pose pose);

		/**
		 * @brief Sets the stance areas for computing the body adjacency map
		 * @param std::vector<SearchArea> stance_areas Stance areas
		 */
		void setStanceAreas(std::vector<SearchArea> stance_areas); //TODO Re-write this method for general settings

		const Pose& getPose() const;

		double getEstimatedGround();

		/**
		 * @brief Gets the stance position of the robot
		 * @return const std::vector<Eigen::Vector2d>& Returns the stance position of each leg
		 */
		const std::vector<Eigen::Vector2d>& getStancePosition() const;

		/**
		 * @brief Gets the body area
		 * @return const Area& Returns the body area
		 */
		const Area& getBodyArea() const;

		/**
		 * @brief Gets the stance areas
		 * @return const std::vector<SearchArea>& Returns the stance areas
		 */
		const std::vector<SearchArea>& getStanceAreas() const; //TODO Get only the stance area of the leg id

		const SearchArea& getLegArea(int leg_id) const;//TODO Use template for the leg and abstract class

		//const SearchArea& getLegArea(HumanoidLegID leg) const;

		/**
		 * @brief Gets the next leg according some defined pattern of locomotion
		 * @param int sequence Sequence number
		 */
		int getNextLeg(int sequence) const;

		/** @brief Gets the number of legs of the robot */
		double getNumberOfLegs() const;


	protected:
		/** @brief Current pose of the robot */
		Pose current_pose_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		/** @brief Vector of the body area */
		Area body_area_;

		/** @brief Vector of stance positions */
		std::vector<Eigen::Vector2d> stance_position_;

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
