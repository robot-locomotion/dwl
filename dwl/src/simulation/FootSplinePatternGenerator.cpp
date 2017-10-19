#include <dwl/simulation/FootSplinePatternGenerator.h>


namespace dwl
{

namespace simulation
{

FootSplinePatternGenerator::FootSplinePatternGenerator() : initial_time_(0.),
		duration_(0.)
{

}


FootSplinePatternGenerator::~FootSplinePatternGenerator()
{

}


void FootSplinePatternGenerator::setParameters(const double& initial_time,
											   const Eigen::Vector3d& initial_pos,
											   const Eigen::Vector3d& target_pos,
											   const StepParameters& params)
{
	// Setting the initial time and duration of the swing movements
	initial_time_ = initial_time;
	duration_ = params.duration;

	// Computing the appex of the swing movement
	Eigen::Vector3d step_delta = target_pos - initial_pos;
	double height_dist = fabs((double) step_delta(rbd::Z));
	double step2d_dist = fabs(step_delta.head<2>().norm());
	double step_theta = atan(height_dist / step2d_dist);
	double target_appex;
	if (target_pos(rbd::Z) >= initial_pos(rbd::Z))
		target_appex = target_pos(rbd::Z) + params.height * cos(step_theta);
	else
		target_appex = initial_pos(rbd::Z) + params.height * cos(step_theta);

	// Setting the spline boundaries
	foot_spliner_x_.setBoundary(initial_time,
								params.duration,
								(double) initial_pos(rbd::X),
								(double) target_pos(rbd::X));
	foot_spliner_y_.setBoundary(initial_time,
								params.duration,
								(double) initial_pos(rbd::Y),
								(double) target_pos(rbd::Y));
	foot_spliner_up_z_.setBoundary(initial_time,
								   params.duration / 2,
								   (double) initial_pos(rbd::Z),
								   target_appex);
	foot_spliner_down_z_.setBoundary(initial_time + params.duration / 2,
									 params.duration / 2,
									 target_appex,
									 (double) target_pos(rbd::Z) -
									 params.penetration);
}


bool FootSplinePatternGenerator::generateTrajectory(Eigen::Vector3d& foot_pos,
													Eigen::Vector3d& foot_vel,
													Eigen::Vector3d& foot_acc,
													const double& time)
{
	if (time < initial_time_)
		return false; // duration it's always positive, and makes sense when
					  // is bigger than the sample time

	// Computing the time that allows us to discriminate the swing-up or swing-down phase
	dwl::math::Spline::Point swing_traj_x, swing_traj_y, swing_traj_z;
	double dt = time - initial_time_;
	foot_spliner_x_.getPoint(time, swing_traj_x);
	foot_spliner_y_.getPoint(time, swing_traj_y);

	if (dt <= (duration_ / 2))
		foot_spliner_up_z_.getPoint(time, swing_traj_z);
	else
		foot_spliner_down_z_.getPoint(time, swing_traj_z);

	// Setting the foot state
	foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
	foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
	foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;

	if (time >= initial_time_ + duration_)
		return false;

	return true;
}


//bool FootSplinePatternGenerator::hapticSwingStopCondition(const Eigen::Matrix3d& Jac,
//														  const Eigen::Vector3d& grforce_base,
//														  double force_th)
//{
//	Eigen::Vector3d surface_normal = Eigen::Vector3d::UnitZ();
//	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
//
//	//compute mobility loss as the min singular value
//	double sigma_min = 1;
//	bool stop_condition = false;
//	Eigen::Matrix3d A;
//	Eigen::JacobiSVD<Eigen::Matrix3d> svd;
//	A = Jac * Jac.transpose();
//	svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
//	sigma_min = (svd.singularValues().array().abs()).minCoeff();
//	double sigma_min_thr = 0.017;
//	bool hit_workspace_lim = sigma_min < sigma_min_thr;
//	if (swingDown){ //check stop condition only at swing down
//		stop_condition = (surface_normal.dot(R.transpose() * grforce_base) >= force_th) || hit_workspace_lim;
//	}
//	//check kin limit
//	if (stop_condition)
//	{
////		swingDown = false;
//		return true;
//	} else
//		return false;
//}

} //@namespace simulation
} //@namespace dwl
