#ifndef DWL_PlaneGrid_H
#define DWL_PlaneGrid_H

#include <Eigen/Dense>
#include <math.h>
#include <utils/utils.h>


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
		 * @param double resolution Resolution of the height
		 */
		PlaneGrid(double gridmap_resolution, double height_resolution);

		/** @brief Destructor function **/
		~PlaneGrid();

		/**
		 * @brief Converts a 2D coordinate into a 2D Key at a certain depth, with boundary checking.
		 * @param dwl::environment::Key& key values that will be computed, an array of fixed size 2.
		 * @param Eigen::Vector2d& coord 2d coordinate of a point
		 * @return true if point is within the planetree (valid), false otherwise
		 */
		bool coordToKeyChecked(Key& key, Eigen::Vector2d coordinate) const;

		/**
		 * @brief Converts a coordinate into a Key at a certain depth, with boundary checking.
		 * @param unsigned short int& key_value Key value that will be computed
		 * @param double coordinate Coordinate of a point
		 * @param bool gridmap Indicates if it is the gridmap
		 * @return true if point is within the planetree (valid), false otherwise
		 */
		bool coordToKeyChecked(unsigned short int& key_value, double coordinate, bool gridmap) const;

		/**
		 * @brief Converts from a single coordinate into a discrete key
		 * @param double coordinate Cartesian coordinate of the cell
		 * @param bool gridmap Indicates if it is the gridmap
		 * @return unsigned short int Return the key of the coordinate
		 */
		unsigned short int coordToKey(double coordinate, bool gridmap) const;

		/**
		 * @brief Converts from a discrete key at the lowest tree level into a coordinate corresponding to the key's center
		 * @param unsigned short int key_value The value of the key
		 * @param bool gridmap Indicates if it is the gridmap
		 * @return double Return the coordinate of the key
		 */
		double keyToCoord(unsigned short int key_value, bool gridmap) const;

		/**
		 * @brief Converts the key of a gridmap to a vertex id
		 * @param dwl::environment::Key gridmap_key Gridmap key
		 * @return Vertex Return the vertex id
		 */
		Vertex gridMapKeyToVertex(Key gridmap_key) const;

		/**
		 * @brief Converts the vertex id to gridmap key
		 * @param Vertex vertex Vertex id
		 * @return dwl::environment::Key Return the gridmap key
		 */
		Key vertexToGridMapKey(Vertex vertex) const;

		/**
		 * @brief Converts 2d coordinate to vertex id
		 * @param Eigen::Vectir2d coordinate 2D coordinate
		 * @return Vertex Return the vertex id
		 */
		Vertex coordToVertex(Eigen::Vector2d coordinate) const;

		/**
		 * @brief Converts vertex id to coordinate
		 * @param Vertex vertex Vertex id
		 * @return Eigen::Vectir2d Return the 2D coordinate
		 */
		Eigen::Vector2d vertexToCoord(Vertex vertex) const;

		/*
		 * @brief Gets the resolution of the gridmap or the height
		 * @param bool gridmap Indicates if it is the gridmap
		 * @return double Return the resolution of the gridmap or height
		 */
		double getResolution(bool gridmap);

		/**
		 * @brief Sets the resolution of the gridmap or the height
		 * @param double resolution Resolution
		 * @param bool gridmap Indicates if it is the gridmap
		 */
		void setResolution(double resolution, bool gridmap);


	private:
		/** @brief The maximun number of grid areas */
		const unsigned int tree_max_val_;

		/** @brief The resolution of the gridmap */
		double gridmap_resolution_;  ///< in meters

		/** @brief The resolution of the z-axis */
		double height_resolution_;

		/** @brief The factor resolution between the minimum resolution and resolutions of others gridmap areas */
		double gridmap_resolution_factor_; ///< = 1. / resolution

		/** @brief The factor resolution between the minimum resolution and resolutions of z-axis */
		double height_resolution_factor_;
};

} //@namespace environment

} //@namespace dwl


#endif




