#ifndef Typedefs_H
#define Typedefs_H

#include <map>
#include <list>
#include <set>

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


#endif
