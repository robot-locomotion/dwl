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


void BodyMotorPrimitives::read(std::string filepath)
{
	std::ifstream fin(filepath.c_str());

	YAML::Parser parser(fin);
	YAML::Node doc;
	while (parser.GetNextDocument(doc)) {
		for (unsigned i = 0; i < doc.size(); i++) {
			BodyMotorPrimitive body_action;
			const YAML::Node& node = doc[i];
			const YAML::Node& subnode = node["action"];

			subnode[0] >> body_action.action(0);
			subnode[1] >> body_action.action(1);
			subnode[2] >> body_action.action(2);
			node["cost"] >> body_action.cost;

			actions_.push_back(body_action);
		}
	}

	is_defined_motor_primitives_ = true;
}


void BodyMotorPrimitives::generateActions(std::vector<Action3d>& actions, Pose3d state)
{
	for (int i = 0; i < (int) actions_.size(); i++)
	{
		// Computing the motor action
		double delta_x = actions_[i].action(0);
		double delta_y = actions_[i].action(1);
		double delta_th = actions_[i].action(2);

		// Computing the current action
		Action3d current_action;
		current_action.pose.position(0) = state.position(0)
				+ delta_x * cos(state.orientation)
				- delta_y * sin(state.orientation);
		current_action.pose.position(1) = state.position(1)
				+ delta_x * sin(state.orientation)
				+ delta_y * cos(state.orientation);
		current_action.pose.orientation = state.orientation + delta_th;
		current_action.cost = actions_[i].cost;

		actions.push_back(current_action);
	}
}

} //@namespace behavior
} //@namespace dwl
