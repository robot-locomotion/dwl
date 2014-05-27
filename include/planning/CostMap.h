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

		virtual void setCostMap();

		virtual void get(AdjacencyMap& state);

		//virtual double get(Eigen::VectorXd state) {return 0;};

	private:
		//std::map<int, std::list<int> > adjacency_node_;
		AdjacencyMap cost_map_;

}; //@class CostMap


} //@namespace planning

} //@namespace dwl



#endif
