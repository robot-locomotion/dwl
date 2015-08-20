#include <model/TerminalStateTrackingEnergyCost.h>


namespace dwl
{

namespace model
{

TerminalStateTrackingEnergyCost::TerminalStateTrackingEnergyCost()
{
	name_ = "terminal state-tracking energy";
}


TerminalStateTrackingEnergyCost::~TerminalStateTrackingEnergyCost()
{

}


void TerminalStateTrackingEnergyCost::compute(double& cost,
											  const LocomotionState& state)
{
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
