#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

WholeBodyDynamics::WholeBodyDynamics() : kin_model_(NULL)
{

}


WholeBodyDynamics::~WholeBodyDynamics()
{

}


void WholeBodyDynamics::setKinematicModel(model::WholeBodyKinematics* kinematics)
{
	kin_model_ = kinematics;
}

} //@namespace model
} //@namespace dwl
