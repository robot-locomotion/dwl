#ifndef DWL_StateCost_H
#define DWL_StateCost_H

#include <planning/Cost.h>


namespace dwl
{

namespace robot
{

class StateCost : public planning::Cost
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

} //@namespace hyq
} //@namespace dwl


#endif
