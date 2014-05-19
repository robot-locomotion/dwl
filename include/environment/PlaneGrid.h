#ifndef DWL_PlaneTree_H
#define DWL_PlaneTree_H

#include <Eigen/Dense>
#include <math.h>


namespace dwl
{

namespace environment
{

/**
 * @brief Struct that defines the id (key) of a certain cell
 */
struct Key
{
	unsigned short int key[2];
};

/**
 * @class PlaneGrid
 * @brief Class for defining the gridmap required for computing a reward map of the terrain
 */
class PlaneGrid
{
	public:
		/**
		 * @brief Constructor function
		 * @param double resolution Resolution of the gridmap
		 */
		PlaneGrid(double resolution);

		/** @brief Destructor function **/
		~PlaneGrid() {}

		/**
		 * @brief Converts a 2D coordinate into a 2D Key at a certain depth, with boundary checking.
		 * @param Eigen::Vector2d& coord 2d coordinate of a point
		 * @param dwl::environment::Key& key values that will be computed, an array of fixed size 2.
		 * @return true if point is within the planetree (valid), false otherwise
		 */
		bool coordToKeyChecked(const Eigen::Vector2d& coord, Key& key) const;

		bool coordToKeyChecked(double coordinate, unsigned short int& keyval) const;

		/**
		 * @brief Converts from a single coordinate into a discrete key
		 * @param double coordinate Cartesian coordinate of the cell
		 */
		unsigned short int coordToKey(double coordinate) const;

		/**
		 * @brief Converts from a discrete key at the lowest tree level into a coordinate corresponding to the key's center
		 * @param unsigned short int key_value The value of the key
		 */
		double keyToCoord(unsigned short int key_value) const;

		/*
		 * @brief Get the resolution of the gridmap
		 */
		double getResolution();

		/**
		 * @brief Set the resolution of the gridmap
		 */
		void setResolution(double resolution);


	private:
		/** @brief The maximun number of grid areas */
		const unsigned int tree_max_val_;

		/** @brief The resolution of the gridmap */
		double resolution_;  ///< in meters

		/** @brief The factor resolution between the minimum resolution and resolutions of others gridmap areas */
		double resolution_factor_; ///< = 1. / resolution

};

} //@namespace environment

} //@namespace dwl


#endif




