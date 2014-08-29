#include <planning/ContactPlanner.h>


namespace dwl
{

namespace planning
{

ContactPlanner::ContactPlanner() : environment_(NULL), robot_(NULL), computation_time_(std::numeric_limits<double>::max()), leg_offset_(0.025)
{

}


ContactPlanner::~ContactPlanner()
{

}


void ContactPlanner::reset(robot::Robot* robot, environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot properties in the contact planner \n" COLOR_RESET);
	robot_ = robot;

	printf(BLUE "Setting the environment information in the contact planner \n" COLOR_RESET);
	environment_ = environment;

	for (int i = 0; i < features_.size(); i++)
		features_[i]->reset(robot);
}


void ContactPlanner::addFeature(environment::Feature* feature)
{
	double weight;
	feature->getWeight(weight);
	printf(GREEN "Adding the %s feature with a weight of %f to the contact planner\n" COLOR_RESET, feature->getName().c_str(), weight);
	features_.push_back(feature);
}


bool ContactPlanner::computeContacts(std::vector<Contact>& footholds, std::vector<Contact> initial_contacts, Pose goal_pose)
{
	// Initilization of footholds
	footholds.clear();

	// Converting quaternion to roll, pitch and yaw angles
	double roll, pitch, yaw;
	Orientation orientation(goal_pose.orientation);
	orientation.getRPY(roll, pitch, yaw);

	// Getting the vertex position
	Eigen::Vector3d body_state;
	body_state << goal_pose.position.head(2), yaw;

	// Getting the terrain cost-map information
	CostMap terrain_costmap;
	environment_->getTerrainCostMap(terrain_costmap);

	// Getting the terrain height-map information
	HeightMap terrain_heightmap;
	environment_->getTerrainHeightMap(terrain_heightmap);

	// Setting the terrain information for computing the reward of the features
	RobotAndTerrain info;
	info.height_map = terrain_heightmap;
	info.resolution = environment_->getTerrainResolution();

	int current_leg_id, past_leg_id = 1;
	for (int l = 0; l < robot_->getNumberOfLegs(); l++) {
		// Computing the current leg according to the predefined pattern of locomotion
		current_leg_id = robot_->getPatternOfLocomotion()[past_leg_id];

		// Setting the current leg as a past leg for future iterations
		past_leg_id = current_leg_id;

		// Computing the current contacts
		std::vector<Contact> current_contacts;
		for (int i = 0; i < robot_->getNumberOfLegs(); i++) {
			if (initial_contacts[i].end_effector != current_leg_id) {
				current_contacts.push_back(initial_contacts[i]);
				std::cout << "Adding " << initial_contacts[i].end_effector << std::endl;
			}
		}
		for (int i = 0; i < robot_->getNumberOfLegs() - 1; i++) {
			for (int j = 0; j < footholds.size(); j++) {
				if (current_contacts[i].end_effector == footholds[j].end_effector) {
					current_contacts[i] = footholds[j];
					std::cout << "Adding " << footholds[j].end_effector << "n" << std::endl;
				}
			}
		}
		info.current_contacts = current_contacts;

		// Computing the boundary of stance area
		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = body_state(0) + robot_->getStanceAreas()[current_leg_id].min_x;
		boundary_min(1) = body_state(1) + robot_->getStanceAreas()[current_leg_id].min_y;
		boundary_max(0) = body_state(0) + robot_->getStanceAreas()[current_leg_id].max_x;
		boundary_max(1) = body_state(1) + robot_->getStanceAreas()[current_leg_id].max_y;
		std::cout << "Current leg = " << current_leg_id << std::endl;

		std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > stance_cost_queue;
		double stance_cost = 0;
		for (double y = boundary_min(1); y < boundary_max(1); y += robot_->getStanceAreas()[current_leg_id].grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += robot_->getStanceAreas()[current_leg_id].grid_resolution) {
				// Computing the rotated coordinate of the point inside the search area
				Eigen::Vector3d current_state;
				current_state(0) = (x - body_state(0)) * cos(yaw) - (y - body_state(1)) * sin(yaw) + body_state(0);
				current_state(1) = (x - body_state(0)) * sin(yaw) + (y - body_state(1)) * cos(yaw) + body_state(1);
				current_state(2) = yaw;

				Vertex current_vertex, terrain_vertex;
				environment_->getTerrainSpaceModel().stateToVertex(current_vertex, current_state);
				environment_->getTerrainSpaceModel().stateVertexToEnvironmentVertex(terrain_vertex, current_vertex, XY_Y);

				// Getting the cost of the terrain
				double terrain_cost, body_cost = 0, contact_cost;
				if (terrain_costmap.find(terrain_vertex)->first == terrain_vertex) {
					terrain_cost = terrain_costmap.find(terrain_vertex)->second;

					// Computing the cost associated with the body
					info.pose.position = goal_pose.position.head(2);
					info.pose.orientation = yaw;
					info.potential_contact.position << current_state.head(2), terrain_heightmap.find(terrain_vertex)->second;
					info.potential_contact.end_effector = current_leg_id;
					for (int i = 0; i < features_.size(); i++) {
						// Computing the cost associated with contact features
						double feature_reward, weight;
						features_[i]->computeReward(feature_reward, info);
						features_[i]->getWeight(weight);

						// Computing the cost of the body feature
						std::cout << features_[i]->getName() << " body cost = -" << weight << " * " << feature_reward << std::endl;
						body_cost -= weight * feature_reward;
					}
					contact_cost = terrain_cost + body_cost;

					// Inserts the contact cost in an organized vertex queue, according to the minimum value
					stance_cost_queue.insert(std::pair<Weight, Vertex>(contact_cost, terrain_vertex));
				}
			}
		}

		Contact foothold;
		foothold.end_effector = current_leg_id;
		if (stance_cost_queue.size() > 0) {
			Vertex foothold_vertex = stance_cost_queue.begin()->second;
			Eigen::Vector2d foothold_coord;
			environment_->getTerrainSpaceModel().vertexToCoord(foothold_coord, foothold_vertex);
			foothold.position << foothold_coord, (terrain_heightmap.find(foothold_vertex)->second + leg_offset_);
		} else {
			Eigen::Vector3d nominal_stance = robot_->getNominalStance()[current_leg_id];
			foothold.position(0) = body_state(0) + nominal_stance(0) * cos(yaw) - nominal_stance(1) * sin(yaw);
			foothold.position(1) = body_state(1) + nominal_stance(0) * sin(yaw) + nominal_stance(1) * cos(yaw);
			foothold.position(2) = robot_->getExpectedGround(current_leg_id);
		}

		footholds.push_back(foothold);
	}

	return true;
}


void ContactPlanner::setComputationTime(double computation_time)
{
	printf("Setting the allowed computation time of the contact solver to %f \n", computation_time);
	computation_time_ = computation_time;
}

} //@namespace planning
} //@namespace dwl
