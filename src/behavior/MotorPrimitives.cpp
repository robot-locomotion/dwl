#include <behavior/MotorPrimitives.h>


namespace dwl
{

namespace behavior
{

MotorPrimitives::MotorPrimitives()
{

}


MotorPrimitives::~MotorPrimitives()
{

}


void MotorPrimitives::generateActions(std::vector<Action3d>& actions, Pose3d state)
{
	printf(YELLOW "Could not generate 3D actions because it is required to define the motor primitives\n" COLOR_RESET);
}

} //@namespace behavior

} //@namespace dwl
