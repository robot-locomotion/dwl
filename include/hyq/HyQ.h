#ifndef DWL_HyQ_H
#define DWL_HyQ_H


namespace dwl {

namespace robot
{


class HyQ : public Robot
{
	public:
		HyQ();
		~HyQ();

		/**
		 * @brief Sets the stance areas for computing the body adjacency map
		 * @param std::vector<SearchArea> stance_areas Stance areas
		 */
		void setStanceAreas(std::vector<SearchArea> stance_areas);


	private:
};


} //@namespace robot

} //@namespace dwl


#endif
