#include <environment/PlaneGrid.h>
#include <iostream>

namespace dwl
{

namespace environment
{


PlaneGrid::PlaneGrid(double gridmap_resolution, double height_resolution) : gridmap_resolution_(gridmap_resolution),
		gridmap_resolution_factor_(1 / gridmap_resolution), height_resolution_(height_resolution), height_resolution_factor_(1 / height_resolution),
		tree_max_val_(32768)
{

}


bool PlaneGrid::coordToKeyChecked(const Eigen::Vector2d& coord, Key& key) const
{
	if (!coordToKeyChecked(key.key[0], (double) coord(0), true))
		return false;
	if (!coordToKeyChecked(key.key[1], (double) coord(1), true))
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


unsigned long int PlaneGrid::gridmapKeyToVertex(Key gridmap_key) const
{
	return (unsigned long int) (gridmap_key.key[1] + std::numeric_limits<unsigned short int>::max() * gridmap_key.key[0]);
}


void PlaneGrid::vertexToGridmapKey(Key& gridmap_key, unsigned long int vertex) const
{
	gridmap_key.key[0] = floor(vertex / std::numeric_limits<unsigned short int>::max());
	gridmap_key.key[1] = vertex - (gridmap_key.key[0] + 1) * std::numeric_limits<unsigned short int>::max();
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
