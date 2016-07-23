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


ReducedBodyState::ReducedBodyState(double _time,
								   Eigen::Vector3d _com_pos,
								   Eigen::Vector3d _ang_pos,
								   Eigen::Vector3d _com_vel,
								   Eigen::Vector3d _ang_vel,
								   Eigen::Vector3d _com_acc,
								   Eigen::Vector3d _ang_acc,
								   Eigen::Vector3d _cop,
								   rbd::BodyPosition _support) :
								 	 time(_time),
									 com_pos(_com_pos),
									 angular_pos(_ang_pos),
									 com_vel(_com_vel),
									 angular_vel(_ang_vel),
									 com_acc(_com_acc),
									 angular_acc(_ang_acc),
									 cop(_cop), support_region(_support)
{

}


ReducedBodyState::~ReducedBodyState()
{

}

} //@namespace dwl
