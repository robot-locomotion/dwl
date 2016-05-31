#ifndef DWL__SIMULATION__LINEAR_CONTROLLED_SLIP_MODEL__H
#define DWL__SIMULATION__LINEAR_CONTROLLED_SLIP_MODEL__H

#include <dwl/utils/DynamicLocomotion.h>
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
	SlipControlParams() : duration(0.) {
		cop_shift.setZero();
	}
	SlipControlParams(double _duration,
					  Eigen::Vector3d _cop_shift) : duration(_duration),
							  cop_shift(_cop_shift) {}
	SlipControlParams(double _duration,
					  Eigen::Vector2d _cop_shift) : duration(_duration),
							  cop_shift(Eigen::Vector3d(_cop_shift(0),
									  	  	  	  	  	_cop_shift(1),
														0.)) {}

	double duration;
	Eigen::Vector3d cop_shift;
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
		 * @param const ReducedBodyState& Initial reduced state
		 * @param const SlipControlParams Slip control parameters
		 */
		void initResponse(const ReducedBodyState& state,
						  const SlipControlParams& params);

		/**
		 * @brief Computes the response of LC-SLIP model
		 * @param ReducedBodyState& Generated reduced state
		 * @param const double& Current time
		 */
		void computeResponse(ReducedBodyState& state,
							 double time);

		/**
		 * @brief Computes the energy associated to the CoM
		 * @param Eigen::Vector3d& CoM energy
		 * @param const ReducedBodyState& Initial reduced state
		 * @param const SlipControlParams& Control parameters
		 */
		void computeSystemEnergy(Eigen::Vector3d& com_energy,
								 const ReducedBodyState& initial_state,
								 const SlipControlParams& params);

	private:
		/** @brief Slip model properties */
		SlipProperties slip_;

		/** @brief Slip control parameters */
		SlipControlParams params_;

		bool init_model_;
		bool init_response_;

		/** @brief Initial state */
		ReducedBodyState initial_state_;

		/** @brief Horizontal dynamic coefficients */
		double slip_omega_;
		Eigen::Vector2d beta_1_;
		Eigen::Vector2d beta_2_;
		Eigen::Vector2d cop_T_;

		/** @brief System energy coefficients */
		Eigen::Vector2d c_1_;
		Eigen::Vector2d c_2_;
		Eigen::Vector2d c_3_;
};

} //@namespace simulation
} //@namespace dwl

#endif
