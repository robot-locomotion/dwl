#include <model/KinematicConstraints.h>


namespace dwl
{

namespace model
{

KinematicConstraints::KinematicConstraints()
{
	name_ = "kinematic";
	is_active_constraint_ = true;
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
