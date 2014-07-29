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
	double lateral_cost = 0.75;
	double diagonal_cost = 0.5;
	double twist_cost = 1;
	motor_costs.push_back(frontal_cost);
	motor_costs.push_back(lateral_cost);
	motor_costs.push_back(diagonal_cost);
	motor_costs.push_back(twist_cost);

	// Defining movements
	BodyMotorPrimitive primitives;
	std::vector<std::vector<Eigen::Vector3d> > motor_actions;
	// Frontal actions
	std::vector<Eigen::Vector3d> frontal_actions;
	frontal_actions.push_back((Eigen::Vector3d) (0.1, 0.0, 0.0));
	frontal_actions.push_back((Eigen::Vector3d) (0.2, 0.0, 0.0));
	frontal_actions.push_back((Eigen::Vector3d) (0.3, 0.0, 0.0));
	frontal_actions.push_back((Eigen::Vector3d) (-0.1, 0.0, 0.0));
	frontal_actions.push_back((Eigen::Vector3d) (-0.2, 0.0, 0.0));
	frontal_actions.push_back((Eigen::Vector3d) (-0.3, 0.0, 0.0));
	motor_actions.push_back(frontal_actions);

	// Lateral actions
	std::vector<Eigen::Vector3d> lateral_actions;
	lateral_actions.push_back((Eigen::Vector3d) (0.0, 0.05, 0.0));
	lateral_actions.push_back((Eigen::Vector3d) (0.0, 0.1, 0.0));
	lateral_actions.push_back((Eigen::Vector3d) (0.0, 0.15, 0.0));
	lateral_actions.push_back((Eigen::Vector3d) (0.0, -0.05, 0.0));
	lateral_actions.push_back((Eigen::Vector3d) (0.0, -0.1, 0.0));
	lateral_actions.push_back((Eigen::Vector3d) (0.0, -0.15, 0.0));
	motor_actions.push_back(lateral_actions);

	// Diagonal actions
	std::vector<Eigen::Vector3d> diagonal_actions;
	diagonal_actions.push_back((Eigen::Vector3d) (0.1, 0.05, 0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (0.1, -0.05, -0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.1, 0.05, -0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.1, -0.05, 0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (0.1, 0.1, 0.7854));
	diagonal_actions.push_back((Eigen::Vector3d) (0.1, -0.1, -0.7854));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.1, 0.1, -0.7854));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.1, -0.1, 0.7854));

	diagonal_actions.push_back((Eigen::Vector3d) (0.2, 0.05, 0.2450));
	diagonal_actions.push_back((Eigen::Vector3d) (0.2, -0.05, -0.2450));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.2, 0.05, -0.2450));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.2, -0.05, 0.2450));
	diagonal_actions.push_back((Eigen::Vector3d) (0.2, 0.1, 0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (0.2, -0.1, -0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.2, 0.1, -0.4636));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.2, -0.1, 0.4636));

	diagonal_actions.push_back((Eigen::Vector3d) (0.3, 0.05, 0.1651));
	diagonal_actions.push_back((Eigen::Vector3d) (0.3, -0.05, -0.1651));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.3, 0.05, -0.1651));
	diagonal_actions.push_back((Eigen::Vector3d) (-0.3, -0.05, 0.1651));
	motor_actions.push_back(diagonal_actions);

	// Twist actions
	std::vector<Eigen::Vector3d> twist_actions;
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, 0.08));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, -0.08));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, 0.16));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, -0.16));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, 0.32));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, -0.32));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, 0.45));
	twist_actions.push_back((Eigen::Vector3d) (0.0, 0.0, -0.45));
	motor_actions.push_back(twist_actions);


	for (int i = 0; i < motor_actions.size(); i++) {
		for (int j = 0; j < motor_actions[i].size(); j++) {
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
