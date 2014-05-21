#include <environment/PlaneGrid.h>
#include <iostream>

namespace dwl
{

namespace environment
{


PlaneGrid::PlaneGrid(double resolution) : resolution_(resolution), resolution_factor_(1 / resolution), tree_max_val_(32768)
{

}


bool PlaneGrid::coordToKeyChecked(const Eigen::Vector2d& coord, Key& key) const
{
	for (int i = 0; i < 2; i++) {
		if (!coordToKeyChecked( (double) coord(i), key.key[i]))
			return false;
	}

	return true;
}


bool PlaneGrid::coordToKeyChecked(double coordinate, unsigned short int& keyval) const
{
	// scale to resolution and shift center for tree_max_val
	int scaled_coord = coordToKey(coordinate);

	// keyval within range of tree?
	if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2 * tree_max_val_))) {
    	keyval = scaled_coord;
    	return true;
	}
	return false;
}


unsigned short int PlaneGrid::coordToKey(double coordinate) const
{
	return ((int) floor(resolution_factor_ * coordinate)) + tree_max_val_;
}


double PlaneGrid::keyToCoord(unsigned short int key_value) const
{
	return ((double) ((int) key_value - (int) this->tree_max_val_) + 0.5) * this->resolution_;
}


double PlaneGrid::getResolution()
{
	return resolution_;
}


void PlaneGrid::setResolution(double resolution)
{
	resolution_ = resolution;
	resolution_factor_ = 1 / resolution;
}


} //@namespace environment

} //@namespace dwl
