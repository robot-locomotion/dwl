#include <robot/StateCost.h>


namespace dwl
{

namespace robot
{

StateCost::StateCost()
{
	name_ = "state";
}


StateCost::~StateCost()
{

}


double StateCost::get(Eigen::VectorXd state)
{
	printf("Getting state cost\n");
	double cost = 0;

	return cost;
}

} //@namespace robot
} //@namespace dwl
