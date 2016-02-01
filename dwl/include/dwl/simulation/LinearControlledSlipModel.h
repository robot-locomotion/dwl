#ifndef DWL__SIMULATION__LINEAR_CONTROLLED_SLIP_MODEL__H
#define DWL__SIMULATION__LINEAR_CONTROLLED_SLIP_MODEL__H

#include <dwl/utils/RigidBodyDynamics.h>
#include <dwl/utils/macros.h>


namespace dwl
{

namespace simulation
{

struct SlipProperties
{
	SlipProperties() : mass(0.), stiffness(0.), gravity(9.81) {}
	SlipProperties(double _mass,
				   double _stiffness,
				   double _gravity = 9.81) : mass(_mass),
						   stiffness(_stiffness), gravity(_gravity) {};

	double mass;
	double stiffness;
	double gravity;
};

struct SlipControlParams
{
	SlipControlParams() : duration(0.), length_shift(0.) {
		cop_shift.setZero();
	}

	SlipControlParams(double _duration,
					  Eigen::Vector3d _cop_shift,
					  double _length_shift) : duration(_duration),
							  cop_shift(_cop_shift),
							  length_shift(_length_shift) {}

	double duration;
	Eigen::Vector3d cop_shift;
	double length_shift;
};

class LinearControlledSlipModel
{
	public:
		LinearControlledSlipModel();
		~LinearControlledSlipModel();

		void setModelProperties(SlipProperties model);
		void initResponse(const double& initial_time,
						  const Eigen::Vector3d& initial_com_pos,
						  const Eigen::Vector3d& initial_com_vel,
						  const Eigen::Vector3d& initial_com_acc,
						  const Eigen::Vector3d& initial_cop,
						  const SlipControlParams& params);

		void computeResponse(Eigen::Vector3d& com_pos,
							 Eigen::Vector3d& com_vel,
							 Eigen::Vector3d& com_acc,
							 Eigen::Vector3d& cop,
							 double time);


	private:
		SlipProperties slip_;
		SlipControlParams params_;

		bool init_model_;
		bool init_response_;

		double initial_time_;
		Eigen::Vector3d initial_com_pos_;
		Eigen::Vector3d initial_com_vel_;
		Eigen::Vector3d initial_com_acc_;
		Eigen::Vector3d initial_cop_;
		double initial_length_;

		double slip_omega_;
		Eigen::Vector2d beta_1_;
		Eigen::Vector2d beta_2_;

		double spring_omega_;
		double d_1_;
		double d_2_;
};

} //@namespace simulation
} //@namespace dwl

#endif
