#include <model/StabilityConstraint.h>


namespace dwl
{

namespace model
{

StabilityConstraint::StabilityConstraint()
{
	name_ = "stability";
}


StabilityConstraint::~StabilityConstraint()
{
	printf("Destroying stability constraint\n");
}


void StabilityConstraint::get(Eigen::VectorXd& constraint, Eigen::VectorXd state)
{
	printf("Getting stability constraints\n");
}

} //@namespace model
} //@namespace dwl
