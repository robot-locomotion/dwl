#include <dwl/behavior/BodyMotorPrimitives.h>


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


void BodyMotorPrimitives::read(std::string filename)
{
	// Yaml reader
	YamlWrapper yaml_reader(filename);

	// Getting the root node
	YAML::Node root;
	yaml_reader.getNode(root);

	// Parsing the configuration file
	for (std::size_t i = 0; i < root.size(); i++) {
		std::string idx = std::to_string(i);
		YamlNamespace ns = {idx};

		BodyMotorPrimitive body_action;
		if (yaml_reader.read(body_action.action, "action", ns) &&
				yaml_reader.read(body_action.cost, "cost", ns))
			actions_.push_back(body_action);
	}

	is_defined_motor_primitives_ = true;
}


void BodyMotorPrimitives::generateActions(std::vector<Action3d>& actions,
										  Pose3d state)
{
	for (unsigned int i = 0; i < actions_.size(); i++) {
		// Computing the motor action
		double delta_x = actions_[i].action(rbd::X);
		double delta_y = actions_[i].action(rbd::Y);
		double delta_th = actions_[i].action(rbd::Z);

		// Computing the current action
		Action3d current_action;
		current_action.pose.position(rbd::X) = state.position(rbd::X)
				+ delta_x * cos(state.orientation)
				- delta_y * sin(state.orientation);
		current_action.pose.position(rbd::Y) = state.position(rbd::Y)
				+ delta_x * sin(state.orientation)
				+ delta_y * cos(state.orientation);
		current_action.pose.orientation = state.orientation + delta_th;
		current_action.cost = actions_[i].cost;

		actions.push_back(current_action);
	}
}

} //@namespace behavior
} //@namespace dwl
