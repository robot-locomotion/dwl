#include <behavior/BodyMotorPrimitives.h>


namespace dwl
{

namespace behavior
{

BodyMotorPrimitives::BodyMotorPrimitives()
{

}

BodyMotorPrimitives::~BodyMotorPrimitives()
{

}


void BodyMotorPrimitives::generateActions(std::vector<Action3d>& actions, Pose3d state)
{
	double delta_x[] = {0.0,  0.0, 0.3, -0.2, 0.15,  0.15, -0.15, -0.15};
	double delta_y[] = {0.1, -0.1, 0.0,  0.0, 0.1, -0.1,  0.1, -0.1};
	double cost[] = {6, 6, 1, 1, 2, 2, 2, 2};

	for (int i = 0; i < 8; i++) {
		Action3d current_action;
		current_action.pose.position(0) = state.position(0) + delta_x[i] * cos(state.orientation) - delta_y[i] * sin(state.orientation);
		current_action.pose.position(1) = state.position(1) + delta_x[i] * sin(state.orientation) + delta_y[i] * cos(state.orientation);

		// Computing the orientation according to the applied action
		if (((delta_y[i] == 0) && (delta_x[i] != 0)) || ((delta_x[i] == 0) && (delta_y[i] != 0)))
			current_action.pose.orientation = state.orientation;
		else
			current_action.pose.orientation = state.orientation + atan2(delta_y[i], delta_x[i]);

		current_action.cost = cost[i];
		actions.push_back(current_action);
	}

	for (int i = 0; i < 8; i++) {
		Action3d current_action;
		current_action.pose.position(0) = state.position(0) + 0.5 * (delta_x[i] * cos(state.orientation) - delta_y[i] * sin(state.orientation));
		current_action.pose.position(1) = state.position(1) + 0.5 * (delta_x[i] * sin(state.orientation) + delta_y[i] * cos(state.orientation));

		// Computing the orientation according to the applied action
		if (((delta_y[i] == 0) && (delta_x[i] != 0)) || ((delta_x[i] == 0) && (delta_y[i] != 0)))
			current_action.pose.orientation = state.orientation + atan2(delta_y[i], delta_x[i]);
		else
			current_action.pose.orientation = state.orientation;

		current_action.cost = cost[i];
		actions.push_back(current_action);
	}

	Action3d current_action;
	current_action.pose.position = state.position;
	current_action.pose.orientation = 0.25;
	actions.push_back(current_action);
	current_action.pose.orientation = -0.25;
	actions.push_back(current_action);
}

} //@namespace behavior
} //@namespace dwl
