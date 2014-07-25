#include <environment/PlaneGrid.h>
#include <iostream>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{

SpaceDiscretization::SpaceDiscretization(double environment_resolution) : environment_resolution_(environment_resolution),
		position_resolution_(0), angular_resolution_(0), max_key_val_(32768)
{

}


SpaceDiscretization::SpaceDiscretization(double environment_resolution, double position_resolution) :
		environment_resolution_(environment_resolution), position_resolution_(position_resolution),
		angular_resolution_(0), max_key_val_(32768)
{

}


SpaceDiscretization::SpaceDiscretization(double environment_resolution, double position_resolution, double angular_resolution) :
		environment_resolution_(environment_resolution), position_resolution_(position_resolution),
		angular_resolution_(angular_resolution), max_key_val_(32768)
{

}


SpaceDiscretization::~SpaceDiscretization()
{

}


bool SpaceDiscretization::coordToKeyChecked(Key& key, const Eigen::Vector3d coordinate) const
{
	if (!coordToKeyChecked(key.x, (double) coordinate(0)))
		return false;
	if (!coordToKeyChecked(key.y, (double) coordinate(1)))
		return false;
	if (!coordToKeyChecked(key.z, (double) coordinate(2)))
		return false;

	return true;
}


bool SpaceDiscretization::coordToKeyChecked(unsigned short int& key, const double coordinate) const
{
	// scale to resolution and shift center for tree_max_val
	unsigned short int scaled_coord;
	coordToKey(scaled_coord, coordinate);

	// keyval within range of tree?
	if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2 * max_key_val_))) {
		key = scaled_coord;
    	return true;
	}

	return false;
}


void SpaceDiscretization::coordToKey(unsigned short int& key, const double coordinate) const
{
	key = (unsigned short int) (floor(coordinate / environment_resolution_) + max_key_val_);
}


void SpaceDiscretization::keyToCoord(double& coordinate, const unsigned short int key) const
{
	coordinate = ((key - max_key_val_) + 0.5) * environment_resolution_;
}


void SpaceDiscretization::keyToVertex(Vertex& vertex, const Key key, const bool plane) const
{
	unsigned long int max_count = std::numeric_limits<unsigned short int>::max() + 1;
	if (plane)
		vertex = key.y + max_count * key.x;
	else
		vertex = key.z + max_count * key.y + max_count * max_count * key.x;
}


void SpaceDiscretization::vertexToKey(Key& key, const Vertex vertex, const bool plane) const
{
	unsigned long int max_count = std::numeric_limits<unsigned short int>::max() + 1;
	if (plane) {
		key.x = floor(vertex / max_count);
		key.y = vertex - max_count * key.x;
	} else {
		key.x = floor(vertex / (max_count * max_count));
		key.y = floor(vertex / max_count) - max_count * max_count * key.x;
		key.z = vertex - max_count * key.y - max_count * max_count * key.x;
	}
}


void SpaceDiscretization::coordToVertex(Vertex& vertex, const Eigen::Vector2d coordinate) const
{
	Key key;
	Eigen::Vector3d coord;
	coord = coordinate, 0;
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
	keyToCoord(x, key.x);
	keyToCoord(y, key.y);

	coordinate(0) = x;
	coordinate(1) = y;
}


void SpaceDiscretization::vertexToCoord(Eigen::Vector3d& coordinate, const Vertex vertex) const
{
	Key key;
	vertexToKey(key, vertex, false);

	double x, y, z;
	keyToCoord(x, key.x);
	keyToCoord(y, key.y);
	keyToCoord(z, key.z);

	coordinate(0) = x;
	coordinate(1) = y;
	coordinate(2) = z;
}


void SpaceDiscretization::stateToKey(unsigned short int& key, const double state, const bool position) const
{
	if (position) {
		if (position_resolution_ == 0)
			printf(RED "Could not the key because it was not defined the position resolution" COLOR_RESET);
		else
			key = (unsigned short int) (floor(state / position_resolution_) + max_key_val_);
	}
	else {
		if (angular_resolution_ == 0)
			printf(RED "Could not the key because it was not defined the angular resolution" COLOR_RESET);
		else {
			if ((state >= (2 * M_PI)) || (state <= 0))
				state -= floor(state / (2 * M_PI)) * (2 * M_PI);

			key = (unsigned short int) (floor(state / angular_resolution_) + max_key_val_);
		}
	}
}


void SpaceDiscretization::keyToState(double& state, const unsigned short int key, const bool position) const
{
	if (position) {
		if (position_resolution_ == 0)
			printf(RED "Could not state of the key because it was not defined the position resolution" COLOR_RESET);
		else
			state = ((double) ((int) key - (int) max_key_val_) + 0.5) * position_resolution_;
	}
	else {
		if (angular_resolution_ == 0)
			printf(RED "Could not the state because it was not defined the angular resolution" COLOR_RESET);
		else
			state = ((double) ((int) key - (int) max_key_val_) + 0.5) * angular_resolution_;
	}
}


void SpaceDiscretization::stateToVertex(Vertex& vertex, const Eigen::Vector2d state) const
{
	// State is defined as (x,y)
	unsigned short int key_x, key_y;
	stateToKey(key_x, (double) state(0), true);
	stateToKey(key_y, (double) state(1), true);

	unsigned long int max_position_count = std::numeric_limits<unsigned short int>::max() + 1;
	unsigned long int max_yaw_count = ceil(2 * M_PI / angular_resolution_) + 1;

	vertex = (unsigned long int) (key_y + (std::numeric_limits<unsigned short int>::max() + 1) * key_x);
}


void SpaceDiscretization::stateToVertex(Vertex& vertex, const Eigen::Vector3d state) const
{
	// State is defined as (x,y,yaw)
	unsigned short int key_x, key_y, key_yaw;
	stateToKey(key_x, (double) state(0), true);
	stateToKey(key_y, (double) state(1), true);
	stateToKey(key_yaw, (double) state(2), false);

	unsigned long int max_position_count = std::numeric_limits<unsigned short int>::max() + 1;
	unsigned long int max_yaw_count = ceil(2 * M_PI / angular_resolution_) + 1;

	vertex = (unsigned long int) key_yaw + max_yaw_count * key_y + max_yaw_count * max_position_count * key_x;
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
	unsigned long int max_position_count = std::numeric_limits<unsigned short int>::max() + 1;
	unsigned int max_yaw_count = ceil(2 * M_PI / angular_resolution_) + 1;
	unsigned short int key_x = floor(vertex / (max_position_count * max_yaw_count));
	unsigned short int key_y = floor(vertex / max_yaw_count) - max_position_count * key_x;
	unsigned short int key_yaw = vertex - max_yaw_count * key_y - max_yaw_count * max_position_count * key_x;

	double x, y, yaw;
	keyToState(x, key_x, true);
	keyToState(y, key_y, true);
	keyToState(yaw, key_yaw, false);

	state(0) = x;
	state(1) = y;
	state(2) = yaw;
}


void SpaceDiscretization::stateVertexToEnvironmentVertex(Vertex& environment_vertex, const Vertex state_vertex, State state) const
{
	switch (state) {
		case XY:
			Eigen::Vector2d state_2d;
			vertexToState(state_2d, state_vertex);
			coordToVertex(environment_vertex, state_2d);
			break;

		case XY_Y:
			Eigen::Vector3d state_3d;
			vertexToState(state_3d, state_vertex);
			Eigen::Vector2d coord = state_3d.head(2);
			coordToVertex(environment_vertex, coord);
			break;

		default:
			printf(RED "Could not computed the environment vertex because it is required to define a compatible definition of state" COLOR_RESET);
			break;
	}
}


double SpaceDiscretization::getEnvironmentResolution()
{
	return environment_resolution_;
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


void SpaceDiscretization::setEnvironmentResolution(double resolution)
{
	environment_resolution_ = resolution;
}


void SpaceDiscretization::setStateResolution(double position_resolution, double angular_resolution)
{
	position_resolution_ = position_resolution;
	if (angular_resolution != 0)
		angular_resolution_ = angular_resolution;
}

} //@namespace environment
} //@namespace dwl
