#ifndef DWL_PlaneTree_H
#define DWL_PlaneTree_H

#include <Eigen/Dense>
#include <math.h>


namespace dwl
{

namespace environment
{


struct Key
{
	unsigned short int key[2];
};

class PlaneGrid
{
	public:
		PlaneGrid(double resolution);
		~PlaneGrid() {}

		/**
		 * Converts a 2D coordinate into a 2D Key at a certain depth, with boundary checking.
		 * @param coord 2d coordinate of a point
		 * @param key values that will be computed, an array of fixed size 2.
		 * @return true if point is within the planetree (valid), false otherwise
		 */
		bool coordToKeyChecked(const Eigen::Vector2d& coord, Key& key) const;

		bool coordToKeyChecked(double coordinate, unsigned short int& keyval) const;

		/// Converts from a single coordinate into a discrete key
		unsigned short int coordToKey(double coordinate) const;

		/// converts from a discrete key at the lowest tree level into a coordinate
		/// corresponding to the key's center
		double keyToCoord(unsigned short int key_value) const;

		double getResolution();

		void setResolution(double resolution);


	private:
		const unsigned int tree_max_val_;
		double resolution_;  ///< in meters
		double resolution_factor_; ///< = 1. / resolution

};

} //@namespace environment

} //@namespace dwl


#endif




