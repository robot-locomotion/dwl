#ifndef DWL__ENVIRONMENT_REPRESENTATION__H
#define DWL__ENVIRONMENT_REPRESENTATION__H

#include <map>
#include <utils/GraphSearching.h>
#include <octomap/octomap.h>


namespace dwl
{

/** @brief Defines if there is an obstacle in a certain vertex for graph-searching algorithms */
typedef std::map<Vertex, bool> ObstacleMap;

/** @brief Defines the height map of the environment */
typedef std::map<Vertex, double> HeightMap;

/**
 * @brief Struct that defines the id (key) of a certain cell
 */
struct Key
{
	Key() : x(0), y(0), z(0) {}
	Key(unsigned short int x_value,
		unsigned short int y_value,
		unsigned short int z_value) : x(x_value), y(y_value), z(z_value) {}
	unsigned short int x;
	unsigned short int y;
	unsigned short int z;
};

/**
 * @brief Struct that defines the information of the cell
 */
struct Cell
{
	Cell() : plane_size(0.), height_size(0.) {}
	Cell(Key key_value, double plane, double height) : key(key_value), plane_size(plane), height_size(height) {}
	Key key;
	double plane_size;
	double height_size;
};

/**
 * @brief Struct that defines the reward information of the cell
 */
struct RewardCell
{
	RewardCell() : reward(0.), plane_size(0.), height_size(0.) {}
	RewardCell(Key key_value, double reward_value, double plane, double height) : key(key_value), reward(reward_value),
			plane_size(plane), height_size(height) {}
	Key key;
	double reward;
	double plane_size;
	double height_size;
};

/**
 * @struct TerrainModel
 * @brief Struct that defines the models of the terrain
 */
struct TerrainModel
{
	octomap::OcTree* octomap;
	//TODO: To integrate others modeler like HeightMap
};

/**
 * @struct Terrain
 * @brief Struct to define the relevant information of the terrain for computing the reward
 */
struct Terrain
{
	Eigen::Vector3d position;
	Eigen::Vector3d surface_normal;
	double curvature;
	std::map<Vertex, double> height_map;
	double min_height;
	double resolution;
};

} //@namespace dwl

#endif
