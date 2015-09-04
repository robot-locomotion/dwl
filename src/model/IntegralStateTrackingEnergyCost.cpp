#include <model/IntegralStateTrackingEnergyCost.h>


namespace dwl
{

namespace model
{

IntegralStateTrackingEnergyCost::IntegralStateTrackingEnergyCost()
{
	name_ = "integral state-tracking energy";
}


IntegralStateTrackingEnergyCost::~IntegralStateTrackingEnergyCost()
{

}


void IntegralStateTrackingEnergyCost::compute(double& cost,
											  const LocomotionState& state)
{
	desired_state_.duration = 0.1;
	if (state.time == 0.1) {
	// 1
	desired_state_.time = 0.1;
	desired_state_.base_pos(dwl::rbd::LZ) = -0.0117982880661;
	desired_state_.base_vel(dwl::rbd::LZ) = -0.117982880661;
	desired_state_.joint_pos << 0.620598290531, -1.54906585111;
	desired_state_.joint_vel << 0.205982905314, -0.490658511111;
//	desired_state_.contacts[0].position << 0.0687062736125, 0.13, -0.582715009885;
//	desired_state_.contacts[0].force << -33.3327717451, 2.19910468131e-09, 94.6053161311;
	}

	if (state.time == 0.2) {
	// 2
	desired_state_.time = 0.2;
	desired_state_.base_pos(dwl::rbd::LZ) = 0.175822198623;
	desired_state_.base_vel(dwl::rbd::LZ) = 1.87620486689;
	desired_state_.joint_pos << 0.0706943272929, -0.349065850399;
	desired_state_.joint_vel << -5.49903963239, 12.0;
//	desired_state_.contacts[0].position << 0.0687062736125, 0.13, -0.582715009885;
//	desired_state_.contacts[0].force << -33.1840702862, 7.27244855798e-09, 344.594877116;
	}

	if (state.time == 0.3) {
	// 3
	desired_state_.time = 0.3;
	desired_state_.base_pos(dwl::rbd::LZ) = 0.246745947708;
	desired_state_.base_vel(dwl::rbd::LZ) = 0.709237490855;
	desired_state_.joint_pos << 0.620598290531, -1.54906585111;//0.261165207304, -1.09366625022;
	desired_state_.joint_vel << 0.709237490855, -7.44600409691;
//	desired_state_.contacts[0].position << 0.161097516953, 0.13, -0.402715009896;
//	desired_state_.contacts[0].force << -16.8628154585, 9.36056317494e-10, 1.01656933023e-07;
	}

	if (state.time == 0.4) {
	// 4
	desired_state_.time = 0.4;
	desired_state_.base_pos(dwl::rbd::LZ) = 0.222272224855;
	desired_state_.base_vel(dwl::rbd::LZ) = -0.24473722853;
	desired_state_.joint_pos <<  0.388034041161, -1.42983410015;
	desired_state_.joint_vel <<1.26868833858, -3.36167849929;
//	desired_state_.contacts[0].position << 0.161097516953, 0.13, -0.355793722729;//-0.402715009896;
//	desired_state_.contacts[0].force << -0.780765656612, 9.36056317494e-10, 1.01656933023e-07;
	}

	if (state.time == 0.5) {
	// 5
	desired_state_.time = 0.5;
	desired_state_.base_pos(dwl::rbd::LZ) = 0.283422656988;
	desired_state_.base_vel(dwl::rbd::LZ) = 0.611504321323;
	desired_state_.joint_pos << 0.170798922223, -0.87678559778;
	desired_state_.joint_vel << -2.17235118938, 5.53048502371;
//	desired_state_.contacts[0].position << 0.161097516953, 0.13, -0.402715009896;
//	desired_state_.contacts[0].force << -63.0045038101, 9.36056317494e-10, 198.291031531;
	}



	// Setting the initial value of the cost
	cost = 0;

	// Computing the base and joint position-tracking error
	if (cost_variables_.base_pos) {
		Eigen::VectorXd base_pos_error = desired_state_.base_pos - state.base_pos;
		cost += base_pos_error.transpose() * locomotion_weights_.base_pos.asDiagonal() * base_pos_error;
	}
	if (cost_variables_.joint_pos) {
		// Checking the joint position size
		if (state.joint_pos.size() != locomotion_weights_.joint_pos.size()) {
			printf(RED "FATAL: the joint position dimensions are not consistent\n" COLOR_RESET);
			exit(EXIT_FAILURE);
		}

		Eigen::VectorXd joint_pos_error = desired_state_.joint_pos - state.joint_pos;
		cost += joint_pos_error.transpose() * locomotion_weights_.joint_pos.asDiagonal() * joint_pos_error;
	}

	// Computing the base and joint velocity-tracking error
	if (cost_variables_.base_vel) {
		Eigen::VectorXd base_vel_error = desired_state_.base_vel - state.base_vel;
		cost += base_vel_error.transpose() * locomotion_weights_.base_vel.asDiagonal() * base_vel_error;
	}
	if (cost_variables_.joint_vel) {
		// Checking the joint velocity size
		if (state.joint_vel.size() != locomotion_weights_.joint_vel.size()) {
			printf(RED "FATAL: the joint velocity dimensions are not consistent\n" COLOR_RESET);
			exit(EXIT_FAILURE);
		}

		Eigen::VectorXd joint_vel_error = desired_state_.joint_vel - state.joint_vel;
		cost += joint_vel_error.transpose() * locomotion_weights_.joint_vel.asDiagonal() * joint_vel_error;
	}

	// Computing the base and joint acceleration-tracking error
	if (cost_variables_.base_acc) {
		Eigen::VectorXd base_acc_error = desired_state_.base_acc - state.base_acc;
		cost += base_acc_error.transpose() * locomotion_weights_.base_acc.asDiagonal() * base_acc_error;
	}
	if (cost_variables_.joint_acc) {
		// Checking the joint acceleration size
		if (state.joint_acc.size() != locomotion_weights_.joint_acc.size()) {
			printf(RED "FATAL: the joint acceleration dimensions are not consistent\n" COLOR_RESET);
			exit(EXIT_FAILURE);
		}

		Eigen::VectorXd joint_acc_error = desired_state_.joint_acc - state.joint_acc;
		cost += joint_acc_error.transpose() * locomotion_weights_.joint_acc.asDiagonal() * joint_acc_error;
	}

	cost *= state.duration;
}

} //@namespace model
} //@namespace dwl
