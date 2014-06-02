#include <environment/PlaneGrid.h>
#include <iostream>
#include <utils/macros.h>

namespace dwl
{

namespace environment
{


PlaneGrid::PlaneGrid(double gridmap_resolution, double height_resolution) : gridmap_resolution_(gridmap_resolution),
		gridmap_resolution_factor_(1 / gridmap_resolution), height_resolution_(height_resolution), height_resolution_factor_(1 / height_resolution),
		tree_max_val_(32768)
{

}


PlaneGrid::~PlaneGrid()
{

}


bool PlaneGrid::coordToKeyChecked(Key& key, Eigen::Vector2d coordinate) const
{
	if (!coordToKeyChecked(key.key[0], (double) coordinate(0), true))
		return false;
	if (!coordToKeyChecked(key.key[1], (double) coordinate(1), true))
		return false;

	return true;
}


bool PlaneGrid::coordToKeyChecked(unsigned short int& key_value, double coordinate, bool gridmap) const
{
	// scale to resolution and shift center for tree_max_val
	int scaled_coord = coordToKey(coordinate, gridmap);

	// keyval within range of tree?
	if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2 * tree_max_val_))) {
		key_value = scaled_coord;
    	return true;
	}
	return false;
}


unsigned short int PlaneGrid::coordToKey(double coordinate, bool gridmap) const
{
	int key;
	if (gridmap)
		key = ((int) floor(gridmap_resolution_factor_ * coordinate)) + tree_max_val_;
	else
		key = ((int) floor(height_resolution_factor_ * coordinate)) + tree_max_val_;

	return key;
}


double PlaneGrid::keyToCoord(unsigned short int key_value, bool gridmap) const
{
	double coord;
	if (gridmap)
		coord = ((double) ((int) key_value - (int) this->tree_max_val_) + 0.5) * this->gridmap_resolution_;
	else
		coord = ((double) ((int) key_value - (int) this->tree_max_val_) + 0.5) * this->height_resolution_;

	return coord;
}


unsigned int PlaneGrid::gridMapKeyToVertex(Key gridmap_key) const
{
	return (unsigned int) (gridmap_key.key[1] + std::numeric_limits<unsigned short int>::max() * gridmap_key.key[0]);
}


Key PlaneGrid::vertexToGridMapKey(Vertex vertex) const
{
	Key gridmap_key;
	gridmap_key.key[0] = floor(vertex / std::numeric_limits<unsigned short int>::max());
	gridmap_key.key[1] = vertex - (gridmap_key.key[0] + 1) * std::numeric_limits<unsigned short int>::max() - 1;

	return gridmap_key;
}


Vertex PlaneGrid::coordToVertex(Eigen::Vector2d coordinate) const
{
	Key key;
	coordToKeyChecked(key, coordinate);

	return gridMapKeyToVertex(key);
}


Eigen::Vector2d PlaneGrid::vertexToCoord(Vertex vertex) const
{
	Key key = vertexToGridMapKey(vertex);

	Eigen::Vector2d coordinate;
	coordinate(0) = keyToCoord(key.key[0], true);
	coordinate(1) = keyToCoord(key.key[1], true);

	return coordinate;
}


double PlaneGrid::getResolution(bool gridmap)
{
	double resolution;
	if (gridmap)
		resolution = gridmap_resolution_;
	else
		resolution = height_resolution_;

	return resolution;
}


void PlaneGrid::setResolution(double resolution, bool gridmap)
{
	if (gridmap) {
		gridmap_resolution_ = resolution;
		gridmap_resolution_factor_ = 1 / resolution;
	} else {
		height_resolution_ = resolution;
		height_resolution_factor_ = 1 / resolution;
	}
}


} //@namespace environment

} //@namespace dwl
