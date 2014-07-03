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

		const std::vector<Eigen::Vector2d>& getStancePosition() const;

		/**
		 * @brief Gets the stance areas
		 * @return std::vector<SearchArea> Returns the stance areas
		 */
		const std::vector<SearchArea>& getStanceAreas() const;

		int getNextLeg(int sequence) const;

		double getNumberOfLegs() const;


	protected:
		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		std::vector<Eigen::Vector2d> stance_position_;

		std::vector<int> pattern_locomotion_;

		double number_legs_;
};


} //@namespace robot
} //@namespace dwl

#endif
