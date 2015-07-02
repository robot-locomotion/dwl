#ifndef DWL_GraphSearching_H
#define DWL_GraphSearching_H

#include <map>
#include <list>


namespace dwl
{

/** @brief Defines a vertex for graph-searching algorithms */
typedef unsigned long int Vertex;

/** @brief Defines a weight for graph-searching algorithms */
typedef double Weight;

/** @brief Defines the cost of a vertex for graph-searching algorithms */
typedef std::map<Vertex, Weight> CostMap;

/** @brief Defines a previous vertex for graph-searching algorithms */
typedef std::map<Vertex, Vertex> PreviousVertex;

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
	Edge() : target(0), weight(0.) {}
	Edge(Vertex target, Weight weight) : target(target), weight(weight) {}

	Vertex target;
	Weight weight;
};

/** Defines an adjacency map for graph-searching algorithms */
typedef std::map<Vertex, std::list<Edge> > AdjacencyMap;


} //@namespace dwl

#endif
