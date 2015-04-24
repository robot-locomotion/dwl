#include <model/StabilityConstraints.h>


namespace dwl
{

namespace model
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

} //@namespace model
} //@namespace dwl
