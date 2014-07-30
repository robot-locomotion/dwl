#ifndef Typedefs_H
#define Typedefs_H

#include <map>
#include <list>
#include <set>
#include <vector>

#include <Eigen/Dense>

#include <octomap/octomap.h>


namespace dwl
{

/** @brief Defines a vertex for graph-searching algorithms */
typedef unsigned long int Vertex;

/** @brief Defines a weight for graph-searching algorithms */
typedef double Weight;

/** @brief Defines the cost of a vertex for graph-searching algorithms */
typedef std::map<Vertex, Weight> CostMap;

/** @brief Defines the height map of the environment */
typedef std::map<Vertex, double> HeightMap;

/** @brief Defines a previous vertex for graph-searching algorithms */
typedef std::map<Vertex, Vertex> PreviousVertex;


enum State {XY, XY_Y, XYZ_RPY};

/**
 * @brief Template struct that orders vertex
 */
template <typename Weight, typename Vertex>
struct pair_first_less
{
    bool operator()(std::pair<Weight,Vertex> vertex_1, std::pair<Weight,Vertex> vertex_2)
    {
        return vertex_1.first < vertex_2.first;
    }
};

/**
 * @brief Defines a edge for graph-searching algorithms
 */
struct Edge
{
	Vertex target;
	Weight weight;
	Edge(Vertex arg_target, Weight arg_weight) : target(arg_target), weight(arg_weight) { }
};

/** Defines an adjacency map for graph-searching algorithms */
typedef std::map<Vertex, std::list<Edge> > AdjacencyMap;

/**
 * @brief Struct that defines the body pose
 */
struct Pose
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation; // Quaternion
};

/**
 * @brief Struct that defines the 3d pose, i.e. (x,y) and yaw angle
 */
struct Pose3d
{
	Eigen::Vector2d position;
	double orientation;
};

/**
 * @brief Struct that defines the 3d actions, i.e. 3d pose and cost for action
 */
struct Action3d
{
	Pose3d pose;
	Weight cost;
};

/**
 * @brief Struct that defines the id (key) of a certain cell
 */
struct Key
{
	unsigned short int x;
	unsigned short int y;
	unsigned short int z;
};

/**
 * @brief Struct that defines the information of the cell
 */
struct Cell
{
	Key key;
	double reward;
	double plane_size;
	double height_size;
};

/**
 * @brief Struct that defines the search area
 */
struct SearchArea
{
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double grid_resolution;
};

/**
 * @struct NeighboringArea
 * @brief Struct that defines the neighboring area
 */
struct NeighboringArea
{
	int min_x, max_x;
	int min_y, max_y;
	int min_z, max_z;
};

/**
 * @brief Struct that defines the models of the terrain
 */
struct TerrainModel
{
	octomap::OcTree* octomap;
	//TODO: To integrate others modeler like HeightMap
};

/**
 * @struct Terrain
 * @brief Struct to define the relevant information of the terrain for computing reward
 */
struct Terrain
{
	Eigen::Vector3d position;
	Eigen::Vector3d surface_normal;
	double curvature;
	std::map<Vertex, double> height_map;
	double resolution;
};

/**
 * @brief Struct that defines a contact
 */
struct Contact
{
	int end_effector;
	Eigen::Vector3d position;
};

} //@namespace dwl


#endif
