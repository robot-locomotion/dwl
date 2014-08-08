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
		 * @brief Sets the stance areas for computing the body adjacency map
		 * @param std::vector<SearchArea> stance_areas Stance areas
		 */
		void setStanceAreas(std::vector<SearchArea> stance_areas); //TODO Re-write this method for general settings

		/**
		 * @brief Gets the stance position of the robot
		 * @return Returns the stance position of each leg
		 */
		const std::vector<Eigen::Vector2d>& getStancePosition() const;

		const SearchArea& getBodyArea() const;

		/**
		 * @brief Gets the stance areas
		 * @return std::vector<SearchArea> Returns the stance areas
		 */
		const std::vector<SearchArea>& getStanceAreas() const;

		/**
		 * @brief Gets the next leg according some defined pattern of locomotion
		 * @param int sequence Sequence number
		 */
		int getNextLeg(int sequence) const;

		/** @brief Gets the number of legs of the robot */
		double getNumberOfLegs() const;


	protected:
		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		/** @brief Vector of the body area */
		SearchArea body_area_;

		/** @brief Vector of stance positions */
		std::vector<Eigen::Vector2d> stance_position_;

		/** @brief Pattern of locomotion */
		std::vector<int> pattern_locomotion_;

		/** @brief Number of legs */
		double number_legs_;
};


} //@namespace robot
} //@namespace dwl

#endif
