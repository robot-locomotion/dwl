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
		rbd::BodyVector foot_pos;
		rbd::BodyVector foot_vel;
		rbd::BodyVector foot_acc;
};

/** @brief Defines a reduced-body trajectory */
typedef std::vector<ReducedBodyState> ReducedBodyTrajectory;

} //@namespace dwl

#endif
