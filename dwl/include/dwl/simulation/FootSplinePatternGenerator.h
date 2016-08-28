#ifndef DWL__SIMULATION__FOOT_SPLINE_PATTERN_GENERATOR__H
#define DWL__SIMULATION__FOOT_SPLINE_PATTERN_GENERATOR__H

#include <dwl/utils/SplineInterpolation.h>
#include <dwl/utils/RigidBodyDynamics.h>
#include <Eigen/Dense>


namespace dwl
{

namespace simulation
{

struct StepParameters
{
	StepParameters() : duration(0.), height(0.), penetration(0.) {}
	StepParameters(double _duration,
				   double _height,
				   double _penetration = 0.) : duration(_duration),
						   height(_height), penetration(_penetration) {}

	/** @brief Duration of the step */
	double duration;

	/** @brief Height of the step */
	double height;

	/** @brief Distance of penetration of the swing trajectory */
	double penetration;
};

class FootSplinePatternGenerator
{
	public:
		/** @brief Constructor function */
		FootSplinePatternGenerator();

		/** @brief Destructor function */
		~FootSplinePatternGenerator();

		/**
		 * @brief Set the parameters for the generation of the foot swing trajectory
		 * This methods assumes that there is not an obstacle in the trajectory.
		 * @param const double& Initial time
		 * @param const Eigen::Vector3d& Initial foot position
		 * @param const Eigen::Vector3d& Target foot position
		 * @param const StepParameters Step parameters
		 */
		void setParameters(const double& initial_time,
						   const Eigen::Vector3d& initial_pos,
						   const Eigen::Vector3d& target_pos,
						   const StepParameters& params);

		/**
		 * @brief Generates the foot-swing trajectory for a given time
		 * @param Eigen::Vector3d& Instantaneous foot position
		 * @param Eigen::Vector3d& Instantaneous foot velocity
		 * @param Eigen::Vector3d& Instantaneous foot acceleration
		 * @param const double& Current time
		 */
		bool generateTrajectory(Eigen::Vector3d& foot_pos,
								Eigen::Vector3d& foot_vel,
								Eigen::Vector3d& foot_acc,
								const double& time);

/*

		bool check_stop_condition(const Eigen::Matrix3d & Jac,
				const Eigen::Vector3d& grforce_base,
				double force_th);
		bool isTimeElapsed(double& t);

*/

	protected:
		/** @brief Spliners for the different axis of the foot movement */
		dwl::math::FifthOrderPolySpline foot_spliner_x_;
		dwl::math::FifthOrderPolySpline foot_spliner_y_;
		dwl::math::FifthOrderPolySpline foot_spliner_up_z_;
		dwl::math::FifthOrderPolySpline foot_spliner_down_z_;

		/** @brief Initial time of the swing trajectory */
		double initial_time_;

		/** @brief Duration of the swing trajectory */
		double duration_;
};

typedef std::map<std::string, FootSplinePatternGenerator> FootSplinerMap;

/*

inline bool FootSplinePatternGenerator::isTimeElapsed(double& t)
{
		if ((t - initial_time_) > duration_)
			return true;
		else
			return false;
}
*/

}
} //@namespace

#endif
