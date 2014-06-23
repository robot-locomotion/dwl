#ifndef DWL_Robot_H
#define DWL_Robot_H

#include <utils/utils.h>


namespace dwl
{

namespace robot
{

class Robot
{
	public:
		Robot();
		~Robot();

		/**
		 * @brief Sets the stance areas for computing the body adjacency map
		 * @param std::vector<SearchArea> stance_areas Stance areas
		 */
		void setStanceAreas(std::vector<SearchArea> stance_areas); //TODO Re-write this method for general settings

		std::vector<SearchArea> getStanceAreas();


	protected:
		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;
};

} //@namespace robot

} //@namespace dwl

#endif
