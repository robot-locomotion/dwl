#include <behavior/BodyMotorPrimitives.h>


namespace dwl
{

namespace behavior
{

BodyMotorPrimitives::BodyMotorPrimitives() //TODO Test a lot with real HyQ
{
//	for (int i = 0; i < (int) motor_actions.size(); i++)
//	{
//		std::cout << "Type of action = " << i << std::endl;
//		for (int j = 0; j < (int) motor_actions[i].size(); j++)
//		{
//			double current_x, next_x, current_y, next_y;
//			current_x = 0.4269;
//			current_y = 0.3886;
//			next_x = motor_actions[i][j](0)
//					+ (current_x * cos((double) motor_actions[i][j](2))
//							- current_y * sin((double) motor_actions[i][j](2)));
//			next_y = motor_actions[i][j](1)
//					+ (current_x * sin((double) motor_actions[i][j](2))
//							+ current_y * cos((double) motor_actions[i][j](2)));
//
//			double distance = pow(
//					pow(next_x - current_x, 2) + pow(next_y - current_y, 2),
//					0.5);
//			std::cout << "Distance of the action = " << distance << std::endl;
//
//			primitives.action = motor_actions[i][j];
//			primitives.cost = motor_costs[i];
//			actions_.push_back(primitives);
//		}
//	}
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
