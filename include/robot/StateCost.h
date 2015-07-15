#ifndef DWL__ROBOT__STATE_COST__H
#define DWL__ROBOT__STATE_COST__H

#include <model/Cost.h>


namespace dwl
{

namespace robot
{

class StateCost : public model::Cost
{
	public:
		/** @brief Constructor function */
		StateCost();

		/** @brief Destructor function */
		virtual ~StateCost();

		//virtual void setCostMap() {}

		virtual double get(Eigen::VectorXd state);

		//virtual void get(planning::AdjacencyMap& adjacency_map) {}
};

} //@namespace robot
} //@namespace dwl


#endif
