#include <hyq/StateCost.h>


namespace dwl
{

namespace hyq
{

StateCost::StateCost()
{
	name_ = "state";
}


void StateCost::get(double cost, Eigen::VectorXd state)
{
	printf("Getting state cost\n");
}

} //@namespace hyq

} //@namespace dwl
