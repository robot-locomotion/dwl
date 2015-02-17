#include <environment/SpaceDiscretization.h>
#include <iostream>


namespace dwl
{

namespace environment
{

SpaceDiscretization::SpaceDiscretization(double environment_resolution) :
		plane_environment_resolution_(environment_resolution),
		height_environment_resolution_(environment_resolution),
		position_resolution_(0), angular_resolution_(0), max_key_val_(32768)
{
	max_key_count_ = std::numeric_limits<unsigned short int>::max() + 1;
	max_position_count_ = std::numeric_limits<unsigned short int>::max() + 1;
	max_angular_count_ = ceil(2 * M_PI / angular_resolution_) + 1;
}


SpaceDiscretization::SpaceDiscretization(double environment_resolution, double position_resolution) :
		plane_environment_resolution_(environment_resolution),
		height_environment_resolution_(environment_resolution), position_resolution_(position_resolution),
		angular_resolution_(0), max_key_val_(32768)
{
	max_key_count_ = std::numeric_limits<unsigned short int>::max() + 1;
	max_position_count_ = std::numeric_limits<unsigned short int>::max() + 1;
	max_angular_count_ = ceil(2 * M_PI / angular_resolution_) + 1;
}


SpaceDiscretization::SpaceDiscretization(double environment_resolution, double position_resolution,
		double angular_resolution) : plane_environment_resolution_(environment_resolution),
				height_environment_resolution_(environment_resolution), position_resolution_(position_resolution),
				angular_resolution_(angular_resolution), max_key_val_(32768)
{
	max_key_count_ = std::numeric_limits<unsigned short int>::max() + 1;
	max_position_count_ = std::numeric_limits<unsigned short int>::max() + 1;
	max_angular_count_ = ceil(2 * M_PI / angular_resolution_) + 1;
}


SpaceDiscretization::~SpaceDiscretization()
{

}


bool SpaceDiscretization::coordToKeyChecked(Key& key, const Eigen::Vector3d coordinate) const
{
	if (!coordToKeyChecked(key.x, (double) coordinate(0), true))
		return false;
	if (!coordToKeyChecked(key.y, (double) coordinate(1), true))
		return false;
	if (!coordToKeyChecked(key.z, (double) coordinate(2), false))
		return false;

	return true;
}


bool SpaceDiscretization::coordToKeyChecked(unsigned short int& key, const double coordinate,
		const bool plane) const
{
	// scale to resolution and shift center for tree_max_val
	unsigned short int scaled_coord;
	coordToKey(scaled_coord, coordinate, plane);

	// keyval within range of tree?
	if ((scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2 * max_key_val_))) {
		key = scaled_coord;
    	return true;
	}

	return false;
}


void SpaceDiscretization::coordToKey(unsigned short int& key, const double coordinate,
		const bool plane) const
{
	if (plane)
		key = (unsigned short int) (floor(coordinate / plane_environment_resolution_) + max_key_val_);
	else
		key = (unsigned short int) (floor(coordinate / height_environment_resolution_) + max_key_val_);
}


void SpaceDiscretization::keyToCoord(double& coordinate, const unsigned short int key,
		const bool plane) const
{
	if (plane)
		coordinate = ((key - max_key_val_) + 0.5) * plane_environment_resolution_;
	else
		coordinate = ((key - max_key_val_) + 0.5) * height_environment_resolution_;
}


void SpaceDiscretization::keyToVertex(Vertex& vertex, const Key key, const bool plane) const
{
	if (plane)
		vertex = key.y + max_key_count_ * key.x;
	else
		vertex = key.z + max_key_count_ * key.y + max_key_count_ * max_key_count_ * key.x;
}


void SpaceDiscretization::vertexToKey(Key& key, const Vertex vertex, const bool plane) const
{
	if (plane) {
		key.x = floor(vertex / max_key_count_);
		key.y = vertex - max_key_count_ * key.x;
	} else {
		key.x = floor(vertex / (max_key_count_ * max_key_count_));
		key.y = floor(vertex / max_key_count_) - max_key_count_ * key.x;
		key.z = vertex - max_key_count_ * key.y - max_key_count_ * max_key_count_ * key.x;
	}
}


void SpaceDiscretization::coordToVertex(Vertex& vertex, const Eigen::Vector2d coordinate) const
{
	Key key;
	Eigen::Vector3d coord;
	coord << coordinate, 0;
	coordToKeyChecked(key, coord);

	keyToVertex(vertex, key, true);
}


void SpaceDiscretization::coordToVertex(Vertex& vertex, const Eigen::Vector3d coordinate) const
{
	Key key;
	coordToKeyChecked(key, coordinate);

	keyToVertex(vertex, key, false);
}


void SpaceDiscretization::vertexToCoord(Eigen::Vector2d& coordinate, const Vertex vertex) const
{
	Key key;
	vertexToKey(key, vertex, true);

	double x, y;
	keyToCoord(x, key.x, true);
	keyToCoord(y, key.y, true);

	coordinate(0) = x;
	coordinate(1) = y;
}


void SpaceDiscretization::vertexToCoord(Eigen::Vector3d& coordinate, const Vertex vertex) const
{
	Key key;
	vertexToKey(key, vertex, false);

	double x, y, z;
	keyToCoord(x, key.x, true);
	keyToCoord(y, key.y, true);
	keyToCoord(z, key.z, false);

	coordinate(0) = x;
	coordinate(1) = y;
	coordinate(2) = z;
}


void SpaceDiscretization::stateToKey(unsigned short int& key, double state, const bool position) const
{
	if (position) {
		if (position_resolution_ == 0)
			printf(RED "Could not get the key because it was not defined the position resolution" COLOR_RESET);
		else
			key = (unsigned short int) (floor(state / position_resolution_) + max_key_val_);
	}
	else {
		if (angular_resolution_ == 0)
			printf(RED "Could not get the key because it was not defined the angular resolution" COLOR_RESET);
		else {
			math::normalizeAngle(state, ZeroTo2Pi);

			unsigned short int max_key_yaw_val_ = 0;
			key = (unsigned short int) (floor(state / angular_resolution_) + max_key_yaw_val_);
		}
	}
}


void SpaceDiscretization::keyToState(double& state, const unsigned short int key, const bool position) const
{
	if (position) {
		if (position_resolution_ == 0)
			printf(RED "Could not state of the key because it was not defined the position resolution\n"
					COLOR_RESET);
		else
			state = ((double) ((int) key - (int) max_key_val_) + 0.5) * position_resolution_;
	}
	else {
		if (angular_resolution_ == 0)
			printf(RED "Could not the state because it was not defined the angular resolution\n"
					COLOR_RESET);
		else {
			unsigned short int max_key_yaw_val_ = 0;
			state = ((double) ((int) key - (int) max_key_yaw_val_)) * angular_resolution_;
		}
	}
}


void SpaceDiscretization::stateToVertex(Vertex& vertex, const Eigen::Vector2d state) const
{
	// State is defined as (x,y)
	unsigned short int key_x, key_y;
	stateToKey(key_x, (double) state(0), true);
	stateToKey(key_y, (double) state(1), true);

	vertex = (unsigned long int) (key_y + max_position_count_ * key_x);
}


void SpaceDiscretization::stateToVertex(Vertex& vertex, const Eigen::Vector3d state) const
{
	// State is defined as (x,y,yaw)
	unsigned short int key_x, key_y, key_yaw;
	stateToKey(key_x, (double) state(0), true);
	stateToKey(key_y, (double) state(1), true);
	stateToKey(key_yaw, (double) state(2), false);

	vertex = (unsigned long int) key_yaw + max_angular_count_ * key_y +
			max_angular_count_ * max_position_count_ * key_x;
}


void SpaceDiscretization::vertexToState(Eigen::Vector2d& state, Vertex vertex) const
{
	unsigned long int max_count = std::numeric_limits<unsigned short int>::max() + 1;
	unsigned short int key_x = floor(vertex / (max_count));
	unsigned short int key_y = vertex - max_count * key_x;

	double x, y;
	keyToState(x, key_x, true);
	keyToState(y, key_y, true);

	state(0) = x;
	state(1) = y;
}


void SpaceDiscretization::vertexToState(Eigen::Vector3d& state, Vertex vertex) const
{
	unsigned short int key_x = floor(vertex / (max_position_count_ * max_angular_count_));
	unsigned short int key_y = floor(vertex / max_angular_count_) - max_position_count_ * key_x;
	unsigned short int key_yaw = vertex - max_angular_count_ * key_y -
			max_angular_count_ * max_position_count_ * key_x;

	double x, y, yaw;
	keyToState(x, key_x, true);
	keyToState(y, key_y, true);
	keyToState(yaw, key_yaw, false);

	state(0) = x;
	state(1) = y;
	state(2) = yaw;
}


void SpaceDiscretization::stateVertexToEnvironmentVertex(Vertex& environment_vertex,
		const Vertex state_vertex, TypeOfState state) const
{
	Eigen::Vector2d state_2d, coord;
	Eigen::Vector3d state_3d;

	switch (state) {
		case XY:
			vertexToState(state_2d, state_vertex);
			coordToVertex(environment_vertex, state_2d);
			break;

		case XY_Y:
			vertexToState(state_3d, state_vertex);
			coord = state_3d.head(2);
			coordToVertex(environment_vertex, coord);
			break;

		default:
			printf(RED "Could not computed the environment vertex because it is required to define a "
					"compatible definition of state" COLOR_RESET);
			break;
	}
}


double SpaceDiscretization::getEnvironmentResolution(const bool plane)
{
	double resolution;
	if (plane)
		resolution = plane_environment_resolution_;
	else
		resolution = height_environment_resolution_;

	return resolution;
}


double SpaceDiscretization::getStateResolution(bool position)
{
	double resolution;
	if (position)
		resolution = position_resolution_;
	else
		resolution = angular_resolution_;

	return resolution;
}


void SpaceDiscretization::setEnvironmentResolution(double resolution, const bool plane)
{
	if (plane)
		plane_environment_resolution_ = resolution;
	else
		height_environment_resolution_ = resolution;
}


void SpaceDiscretization::setStateResolution(double position_resolution, double angular_resolution)
{
	position_resolution_ = position_resolution;
	if (angular_resolution != 0) {
		angular_resolution_ = angular_resolution;
		max_angular_count_ = ceil(2 * M_PI / angular_resolution_) + 1;
	}
}

} //@namespace environment
} //@namespace dwl
