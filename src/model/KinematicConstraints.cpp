#include <model/KinematicConstraints.h>


namespace dwl
{

namespace model
{

KinematicConstraints::KinematicConstraints()
{
	name_ = "kinematic";
}


KinematicConstraints::~KinematicConstraints()
{
	printf("Destroying kinematic constraint\n");
}


void KinematicConstraints::get(Eigen::VectorXd& constraint, Eigen::VectorXd state)
{
	printf("Getting kinematic constraints\n");
}

} //@namespace model
} //@namespace dwl
