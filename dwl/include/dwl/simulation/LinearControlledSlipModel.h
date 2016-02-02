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

/**
 * @class LinearControlledSlipModel
 * @brief Describes the response of linear-controlled SLIP model
 * This class describes a response of the Linear-Controlled Spring
 * Loaded Inverted Pendulum (LC-SLIP) model. This model decouples
 * the horizontal and vertical dynamics of the system. Additionally,
 * it uses a linear-control policy for the displacement of the
 * Center of Pressure (CoP) and pendulum length.
 */
class LinearControlledSlipModel
{
	public:
		/** @brief Constructor function */
		LinearControlledSlipModel();

		/** @brief Destructor function */
		~LinearControlledSlipModel();

		/**
		 * @brief Set the model properties
		 * The LC-SLIP model properties are the mass, pendulum
		 * stiffness and gravity.
		 * @param SlipProperties Slip model properties
		 */
		void setModelProperties(SlipProperties model);

		/**
		 * @brief Initializes the parameters for the computing the response
		 * @param const double& Initial time
		 * @param const Eigen::Vector3d& Initial CoM position
		 * @param const Eigen::Vector3d& Initial CoM velocity
		 * @param const Eigen::Vector3d& Initial CoM acceleration
		 * @param const Eigen::Vector3d& Initial CoP position
		 * @param const SlipControlParams Slip control parameters
		 */
		void initResponse(const double& initial_time,
						  const Eigen::Vector3d& initial_com_pos,
						  const Eigen::Vector3d& initial_com_vel,
						  const Eigen::Vector3d& initial_com_acc,
						  const Eigen::Vector3d& initial_cop,
						  const SlipControlParams& params);

		/**
		 * @brief Computes the response of LC-SLIP model
		 * @param Eigen::Vector3d& Instantaneous CoM position
		 * @param Eigen::Vector3d& Instantaneous CoM velocity
		 * @param Eigen::Vector3d& Instantaneous CoM acceleration
		 * @param Eigen::Vector3d& Instantaneous CoP position
		 * @param const double& Current time
		 */
		void computeResponse(Eigen::Vector3d& com_pos,
							 Eigen::Vector3d& com_vel,
							 Eigen::Vector3d& com_acc,
							 Eigen::Vector3d& cop,
							 double time);


	private:
		/** @brief Slip model properties */
		SlipProperties slip_;

		/** @brief Slip control parameters */
		SlipControlParams params_;

		bool init_model_;
		bool init_response_;

		/** @brief Initial state */
		double initial_time_;
		Eigen::Vector3d initial_com_pos_;
		Eigen::Vector3d initial_com_vel_;
		Eigen::Vector3d initial_com_acc_;
		Eigen::Vector3d initial_cop_;
		double initial_length_;

		/** @brief Horizontal dynamic coefficients */
		double slip_omega_;
		Eigen::Vector2d beta_1_;
		Eigen::Vector2d beta_2_;

		/** @brief Vertical dynamic coefficients */
		double spring_omega_;
		double d_1_;
		double d_2_;
};

} //@namespace simulation
} //@namespace dwl

#endif
