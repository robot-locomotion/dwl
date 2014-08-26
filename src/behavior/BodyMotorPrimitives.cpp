#include <behavior/BodyMotorPrimitives.h>


namespace dwl
{

namespace behavior
{

BodyMotorPrimitives::BodyMotorPrimitives()
{
	// Defining the cost per movement class
	std::vector<double> motor_costs;
	double frontal_cost = 0.5;
	double lateral_cost = 1.0;
	double diagonal_cost = 0.5;
	double twist_cost = 1.0;
	motor_costs.push_back(frontal_cost);
	motor_costs.push_back(lateral_cost);
	motor_costs.push_back(diagonal_cost);
	motor_costs.push_back(twist_cost);

	// Defining movements
	BodyMotorPrimitive primitives;
	std::vector<std::vector<Eigen::Vector3d> > motor_actions;
	// Frontal actions
	std::vector<Eigen::Vector3d> frontal_actions;
	Eigen::Vector3d action;
	action << 0.05, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << 0.1, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << 0.175, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << 0.235, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << -0.05, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << -0.1, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << -0.175, 0.0, 0.0;
	frontal_actions.push_back(action);
	action << -0.235, 0.0, 0.0;
	frontal_actions.push_back(action);
	motor_actions.push_back(frontal_actions);

	// Lateral actions
	std::vector<Eigen::Vector3d> lateral_actions;
	action << 0.0, 0.05, 0.0;
	lateral_actions.push_back(action);
	action << 0.0, 0.1, 0.0;
	lateral_actions.push_back(action);
	action << 0.0, 0.125, 0.0;
	lateral_actions.push_back(action);
	action << 0.0, -0.05, 0.0;
	lateral_actions.push_back(action);
	action << 0.0, -0.1, 0.0;
	lateral_actions.push_back(action);
	action << 0.0, -0.125, 0.0;
	lateral_actions.push_back(action);
	motor_actions.push_back(lateral_actions);

	// Diagonal actions
	std::vector<Eigen::Vector3d> diagonal_actions;
	action << 0.015, 0.01, 0.5880;
	diagonal_actions.push_back(action);
	action << 0.015, -0.01, -0.5880;
	diagonal_actions.push_back(action);
	action << -0.015, 0.01, -0.5880;
	diagonal_actions.push_back(action);
	action << -0.015, -0.01, 0.5880;
	diagonal_actions.push_back(action);

	action << 0.035, 0.02, 0.5191;
	diagonal_actions.push_back(action);
	action << 0.035, -0.02, -0.5191;
	diagonal_actions.push_back(action);
	action << -0.035, 0.02, -0.5191;
	diagonal_actions.push_back(action);
	action << -0.035, -0.02, 0.5191;
	diagonal_actions.push_back(action);

	action << 0.035, 0.017, 0.4522;
	diagonal_actions.push_back(action);
	action << 0.035, -0.017, -0.4522;
	diagonal_actions.push_back(action);
	action << -0.035, 0.017, -0.4522;
	diagonal_actions.push_back(action);
	action << -0.035, -0.017, 0.4522;
	diagonal_actions.push_back(action);

	action << 0.1, 0.035, 0.3367;
	diagonal_actions.push_back(action);
	action << 0.1, -0.035, -0.3367;
	diagonal_actions.push_back(action);
	action << -0.1, 0.035, -0.3367;
	diagonal_actions.push_back(action);
	action << -0.1, -0.035, 0.3367;
	diagonal_actions.push_back(action);

	action << 0.15, 0.055, 0.3514;
	diagonal_actions.push_back(action);
	action << 0.15, -0.055, -0.3514;
	diagonal_actions.push_back(action);
	action << -0.15, 0.055, -0.3514;
	diagonal_actions.push_back(action);
	action << -0.15, -0.055, 0.3514;
	diagonal_actions.push_back(action);

	action << 0.175, 0.06, 0.3303;
	diagonal_actions.push_back(action);
	action << 0.175, -0.06, -0.3303;
	diagonal_actions.push_back(action);
	action << -0.175, 0.06, -0.3303;
	diagonal_actions.push_back(action);
	action << -0.175, -0.06, 0.3303;
	diagonal_actions.push_back(action);

	action << 0.2, 0.04, 0.1973;
	diagonal_actions.push_back(action);
	action << 0.2, -0.04, -0.1973;
	diagonal_actions.push_back(action);
	action << -0.2, 0.04, -0.1973;
	diagonal_actions.push_back(action);
	action << -0.2, -0.04, 0.1973;
	diagonal_actions.push_back(action);

	action << 0.25, 0.05, 0.1974;
	diagonal_actions.push_back(action);
	action << 0.25, -0.05, -0.1974;
	diagonal_actions.push_back(action);
	action << -0.25, 0.05, -0.1974;
	diagonal_actions.push_back(action);
	action << -0.25, -0.05, 0.1974;
	diagonal_actions.push_back(action);

	action << 0.275, 0.05, 0.1799;
	diagonal_actions.push_back(action);
	action << 0.275, -0.05, -0.1799;
	diagonal_actions.push_back(action);
	action << -0.275, 0.05, -0.1799;
	diagonal_actions.push_back(action);
	action << -0.275, -0.05, 0.1799;
	diagonal_actions.push_back(action);

	action << 0.3, 0.035, 0.1161;
	diagonal_actions.push_back(action);
	action << 0.3, -0.035, -0.1161;
	diagonal_actions.push_back(action);
	action << -0.3, 0.035, -0.1161;
	diagonal_actions.push_back(action);
	action << -0.3, -0.035, 0.1161;
	diagonal_actions.push_back(action);

	motor_actions.push_back(diagonal_actions);

	// Twist actions
	std::vector<Eigen::Vector3d> twist_actions;
	action << 0.0, 0.0, 0.04;
	twist_actions.push_back(action);
	action << 0.0, 0.0, -0.04;
	twist_actions.push_back(action);
	action << 0.0, 0.0, 0.08;
	twist_actions.push_back(action);
	action << 0.0, 0.0, -0.08;
	twist_actions.push_back(action);
	action << 0.0, 0.0, 0.16;
	twist_actions.push_back(action);
	action << 0.0, 0.0, -0.16;
	twist_actions.push_back(action);
	action << 0.0, 0.0, 0.24;
	twist_actions.push_back(action);
	action << 0.0, 0.0, -0.24;
	twist_actions.push_back(action);
	action << 0.0, 0.0, 0.32;
	twist_actions.push_back(action);
	action << 0.0, 0.0, -0.32;
	twist_actions.push_back(action);
	action << 0.0, 0.0, 0.45;
	twist_actions.push_back(action);
	action << 0.0, 0.0, -0.45;
	twist_actions.push_back(action);
	motor_actions.push_back(twist_actions);


	for (int i = 0; i < motor_actions.size(); i++) {
		std::cout << "Type of action = " << i << std::endl;
		for (int j = 0; j < motor_actions[i].size(); j++) {
			double current_x, next_x, current_y, next_y;
			current_x = 0.4269;
			current_y = 0.3886;
			next_x = motor_actions[i][j](0) + (current_x * cos((double) motor_actions[i][j](2)) - current_y * sin((double) motor_actions[i][j](2)));
			next_y = motor_actions[i][j](1) + (current_x * sin((double) motor_actions[i][j](2)) + current_y * cos((double) motor_actions[i][j](2)));

			double distance = pow(pow(next_x - current_x, 2) + pow(next_y - current_y, 2), 0.5);
			std::cout << "Distance of the action = " << distance << std::endl;

			primitives.action = motor_actions[i][j];
			primitives.cost = motor_costs[i];
			actions_.push_back(primitives);
		}
	}
}


BodyMotorPrimitives::~BodyMotorPrimitives()
{

}


void BodyMotorPrimitives::generateActions(std::vector<Action3d>& actions, Pose3d state)
{
	for (int i = 0; i < actions_.size(); i++) {
		// Computing the motor action
		double delta_x = actions_[i].action(0);
		double delta_y = actions_[i].action(1);
		double delta_th = actions_[i].action(2);

		// Computing the current action
		Action3d current_action;
		current_action.pose.position(0) = state.position(0) + delta_x * cos(state.orientation) - delta_y * sin(state.orientation);
		current_action.pose.position(1) = state.position(1) + delta_x * sin(state.orientation) + delta_y * cos(state.orientation);
		current_action.pose.orientation = state.orientation + delta_th;
		current_action.cost = actions_[i].cost;

		actions.push_back(current_action);
	}
}

} //@namespace behavior
} //@namespace dwl
