#include <robot/HyQWholeBodyDynamics.h>


namespace dwl
{

namespace robot
{

HyQWholeBodyDynamics::HyQWholeBodyDynamics() : id_(inertia_, motion_tf_)
{

}


HyQWholeBodyDynamics::~HyQWholeBodyDynamics()
{

}


} //@namespace robot
} //namespace dwl
