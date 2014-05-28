#ifndef DWL_CostMap_H
#define DWL_CostMap_H

#include <planning/Cost.h>


namespace dwl
{

namespace planning
{


class CostMap : public Cost
{
	public:
		CostMap();
		virtual ~CostMap();

		virtual void setCostMap(std::vector<dwl::environment::Cell> reward_map);

		virtual void get(AdjacencyMap& state);


	private:
		/** @brief Adjacency map that defines a cost map of the environment */
		AdjacencyMap cost_map_;

}; //@class CostMap


} //@namespace planning

} //@namespace dwl



#endif
