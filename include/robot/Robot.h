#ifndef DWL_Robot_H
#define DWL_Robot_H

#include <utils/utils.h>


namespace dwl
{

namespace robot
{

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
		 * @brief Gets the stance areas
		 * @return std::vector<SearchArea> Returns the stance areas
		 */
		std::vector<SearchArea> getStanceAreas();


	protected:
		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;
};


} //@namespace robot
} //@namespace dwl

#endif
