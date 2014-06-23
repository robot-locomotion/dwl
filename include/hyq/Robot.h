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
		virtual ~Robot();

		/**
		 * @brief Abstract method for setting the stance areas for computing the body adjacency map
		 * @param std::vector<SearchArea> stance_areas Stance areas
		 */
		virtual void setStanceAreas(std::vector<SearchArea> stance_areas) = 0;


	protected:
		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;
};

} //@namespace robot

} //@namespace dwl

#endif
