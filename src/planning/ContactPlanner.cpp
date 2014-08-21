#include <planning/ContactPlanner.h>


namespace dwl
{

namespace planning
{

ContactPlanner::ContactPlanner() : environment_(NULL), robot_(NULL), computation_time_(std::numeric_limits<double>::max())
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
}


bool ContactPlanner::computeFootholds(std::vector<Contact>& footholds, Pose current_pose)
{
	// Converting quaternion to roll, pitch and yaw angles
	double roll, pitch, yaw;
	Orientation orientation(current_pose.orientation);
	orientation.getRPY(roll, pitch, yaw);

	// Getting the vertex position
	Eigen::Vector3d body_state;
	body_state << current_pose.position.head(2), yaw;

	// Getting the terrain cost map information
	CostMap terrain_costmap;
	environment_->getTerrainCostMap(terrain_costmap);

	// Getting the terrain height map information
	HeightMap terrain_heightmap;
	environment_->getTerrainHeightMap(terrain_heightmap);

	for (int leg = 0; leg < robot_->getNumberOfLegs(); leg++) {
		int current_leg_id = robot_->getPatternOfLocomotion()[leg];

		double body_cost;
		// Computing the boundary of stance area
		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = robot_->getStanceAreas()[current_leg_id].min_x + body_state(0);
		boundary_min(1) = robot_->getStanceAreas()[current_leg_id].min_y + body_state(1);
		boundary_max(0) = robot_->getStanceAreas()[current_leg_id].max_x + body_state(0);
		boundary_max(1) = robot_->getStanceAreas()[current_leg_id].max_y + body_state(1);

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

				// Inserts the element in an organized vertex queue, according to the maximun value
				if (terrain_costmap.find(terrain_vertex)->first == terrain_vertex)
					stance_cost_queue.insert(std::pair<Weight, Vertex>(terrain_costmap.find(terrain_vertex)->second, terrain_vertex));
			}
		}

		Contact foothold;
		foothold.end_effector = current_leg_id;
		if (stance_cost_queue.size() > 0) {
			Vertex foothold_vertex = stance_cost_queue.begin()->second;
			Eigen::Vector2d coord;
			environment_->getTerrainSpaceModel().vertexToCoord(coord, foothold_vertex);
			foothold.position << coord, terrain_heightmap.find(foothold_vertex)->second;
		}
		else {
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
