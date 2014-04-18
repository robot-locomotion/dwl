#ifndef DWL_HyQ_StateCost_H
#define DWL_HyQ_StateCost_H

#include <planning/Cost.h>



namespace dwl
{

namespace hyq
{

class StateCost : public dwl::planning::Cost
{
	public:
		/** @brief Constructor function */
		StateCost();

		/** @brief Destructor function */
		~StateCost() {}

		void get(double cost, Eigen::VectorXd state);

};

} //@namespace hyq

} //@namespace dwl


#endif
