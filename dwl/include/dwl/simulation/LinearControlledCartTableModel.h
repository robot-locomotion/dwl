#ifndef DWL__SIMULATION__LINEAR_CONTROLLED_SLIP_MODEL__H
#define DWL__SIMULATION__LINEAR_CONTROLLED_SLIP_MODEL__H

#include <dwl/utils/DynamicLocomotion.h>
#include <dwl/utils/RigidBodyDynamics.h>
#include <dwl/utils/macros.h>


namespace dwl
{

namespace simulation
{

struct CartTableProperties
{
	CartTableProperties() : mass(0.), gravity(9.81) {}
	CartTableProperties(double _mass,
						double _gravity = 9.81) : mass(_mass),
								gravity(_gravity) {};

	double mass;
	double gravity;
};

struct CartTableControlParams
{
	CartTableControlParams() : duration(0.) {
		cop_shift.setZero();
	}
	CartTableControlParams(double _duration,
						   Eigen::Vector3d _cop_shift) : duration(_duration),
							  cop_shift(_cop_shift) {}
	CartTableControlParams(double _duration,
						   Eigen::Vector2d _cop_shift) : duration(_duration),
							  cop_shift(Eigen::Vector3d(_cop_shift(0),
									  	  	  	  	  	_cop_shift(1),
														0.)) {}

	double duration;
	Eigen::Vector3d cop_shift;
};

/**
 * @class LinearControlledCartTableModel
 * @brief Describes the response of linear-controlled cart-table model
 * This class describes a response of the Linear-Controlled Cart-Table
 * (LC-CT) model. This model considers just the horizontal dynamics of
 * the system. Additionally, it uses a linear-control policy for the
 * displacement of the Center of Pressure (CoP).
 */
class LinearControlledCartTableModel
{
	public:
		/** @brief Constructor function */
		LinearControlledCartTableModel();

		/** @brief Destructor function */
		~LinearControlledCartTableModel();

		/**
		 * @brief Set the model properties
		 * The cart-table model properties are the mass and gravity
		 * @param CartTableProperties Cart-table model properties
		 */
		void setModelProperties(CartTableProperties model);

		/**
		 * @brief Initializes the parameters for the computing the response
		 * @param const ReducedBodyState& Initial reduced state
		 * @param const CartTableControlParams Cart-table control parameters
		 */
		void initResponse(const ReducedBodyState& state,
						  const CartTableControlParams& params);

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
		 * @param const CartTableControlParams& Control parameters
		 */
		void computeSystemEnergy(Eigen::Vector3d& com_energy,
								 const ReducedBodyState& initial_state,
								 const CartTableControlParams& params);

		/** @brief Gets the pendulum height */
		double getPendulumHeight();


	private:
		/** @brief Cart-table model properties */
		CartTableProperties properties_;

		/** @brief Cart-table control parameters */
		CartTableControlParams params_;

		bool init_model_;
		bool init_response_;

		/** @brief Initial state */
		ReducedBodyState initial_state_;

		/** @brief Horizontal dynamic coefficients */
		double height_;
		double omega_;
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
