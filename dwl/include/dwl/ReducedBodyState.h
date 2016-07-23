#ifndef DWL__REDUCED_BODY_STATE__H
#define DWL__REDUCED_BODY_STATE__H

#include <Eigen/Dense>
#include <map>
#include <vector>

#include <dwl/utils/Orientation.h>
#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

class ReducedBodyState
{
	public:
		ReducedBodyState();
		ReducedBodyState(double _time,
						 Eigen::Vector3d _com_pos,
						 Eigen::Vector3d _ang_pos,
						 Eigen::Vector3d _com_vel,
						 Eigen::Vector3d _ang_vel,
						 Eigen::Vector3d _com_acc,
						 Eigen::Vector3d _ang_acc,
						 Eigen::Vector3d _cop,
						 rbd::BodyPosition _support);

		~ReducedBodyState();


		double time;
		Eigen::Vector3d com_pos;
		Eigen::Vector3d angular_pos;
		Eigen::Vector3d com_vel;
		Eigen::Vector3d angular_vel;
		Eigen::Vector3d com_acc;
		Eigen::Vector3d angular_acc;
		Eigen::Vector3d cop;
		rbd::BodyPosition support_region;
};

/** @brief Defines a reduced-body trajectory */
typedef std::vector<ReducedBodyState> ReducedBodyTrajectory;

} //@namespace dwl

#endif
