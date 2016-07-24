#include <dwl/ReducedBodyState.h>


namespace dwl
{

ReducedBodyState::ReducedBodyState() : time(0.)
{
	com_pos.setZero();
	angular_pos.setZero();
	com_vel.setZero();
	angular_vel.setZero();
	com_acc.setZero();
	angular_acc.setZero();
	cop.setZero();
}


ReducedBodyState::~ReducedBodyState()
{

}

} //@namespace dwl
