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


void BodyMotorPrimitives::computeAction(std::vector<Pose3d>& actions, Pose3d state)
{
	double delta_x[] = {0.0,  0.0, 0.3, -0.2, 0.15,  0.15, -0.15, -0.15};
	double delta_y[] = {0.1, -0.1, 0.0,  0.0, 0.1, -0.1,  0.1,  0.1};

	for (int i = 0; i < 8; i++) {
		Pose3d current_action;
		double cosine = cos(delta_y[i] / delta_x[i]) + state.orientation;
		double sine = sin(delta_y[i] / delta_x[i]) + state.orientation;
		current_action.position(0) = state.position(0) + delta_x[i] * cosine - delta_y[i] * sine;
		current_action.position(1) = state.position(1) + delta_x[i] * sine + delta_y[i] * cosine;
		current_action.orientation = state.orientation + atan2(delta_y[i], delta_x[i]);
		actions.push_back(current_action);
	}

	for (int i = 0; i < 8; i++) {
		Pose3d current_action;
		double cosine = cos(delta_y[i] / delta_x[i]) + state.orientation;
		double sine = sin(delta_y[i] / delta_x[i]) + state.orientation;
		current_action.position(0) = state.position(0) + 0.5 * (delta_x[i] * cosine - delta_y[i] * sine);
		current_action.position(1) = state.position(1) + 0.5 * (delta_x[i] * sine + delta_y[i] * cosine);
		current_action.orientation = state.orientation + atan2(delta_y[i], delta_x[i]);
		actions.push_back(current_action);
	}
}

} //@namespace behavior
} //@namespace dwl
