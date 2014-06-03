#ifndef Typedefs_H
#define Typedefs_H

#include <map>
#include <list>
#include <set>


namespace dwl
{

/** Defines a vertex for graph-searching algorithms */
typedef unsigned int Vertex;

/** Defines a weight for graph-searching algorithms */
typedef double Weight;

/** Defines the cost of a vertex for graph-searching algorithms */
typedef std::map<Vertex, Weight> VertexCost;

/** Defines a previous vertex for graph-searching algorithms */
typedef std::map<Vertex, Vertex> PreviousVertex;

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
	Eigen::Vector4d orientation; // Quaternion
};

/**
 * @brief Struct that defines the id (key) of a certain cell
 */
struct Key
{
	unsigned short int key[2];
};

/**
 * @brief Struct that defines the id (key) of a certain cell
 */
struct CellKey
{
	Key grid_id;
	unsigned short int height_id;
};

/**
 * @brief Struct that defines the information of the cell
 */
struct Cell
{
	CellKey cell_key;
	double reward;
	double size;
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

} //@namespace dwl


#endif
