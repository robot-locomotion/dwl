#include <constraint/StabilityConstraints.h>


namespace dwl
{

namespace constraint
{

StabilityConstraints::StabilityConstraints()
{
	name_ = "stability";
	is_active_constraint_ = true;
}


StabilityConstraints::~StabilityConstraints()
{
	printf("Destroying stability constraint\n");
}


void StabilityConstraints::get(Eigen::VectorXd& constraint, Eigen::VectorXd state)
{
	printf("Getting stability constraints\n");
}

} //@namespace constraint
} //@namespace dwl
