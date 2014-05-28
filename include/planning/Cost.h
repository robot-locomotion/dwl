#ifndef DWL_Cost_H
#define DWL_Cost_H

#include <environment/RewardMap.h>
#include <environment/PlaneGrid.h>
#include <Eigen/Dense>
#include <utils/macros.h>

#include <vector>
#include <map>
#include <list>
#include <set>


namespace dwl
{

namespace planning
{

/** Defines a vertex for graph-searching algorithms */
typedef unsigned int Vertex;

/** Defines a weight for graph-searching algorithms */
typedef double Weight;

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

/** Defines the cost of a vertex for graph-searching algorithms */
typedef std::map<Vertex, Weight> VertexCost;

/** Defines a previous vertex for graph-searching algorithms */
typedef std::map<Vertex, Vertex> PreviousVertex;

/**
 * @class Cost
 * @brief Abstract class for computing the cost of the planning of motion sequence problem (optimization problem)
 */
class Cost
{
	public:
		/** @brief Constructor function */
		Cost();

		/** @brief Destructor function */
		virtual ~Cost();

		/**
		 * @brief Sets the cost map only for Cost that representing the terrain (or map)
		 * @param std::vector<dwl::environment::Cell> reward_map Reward map
		 */
		virtual void setCostMap(std::vector<dwl::environment::Cell> reward_map);

		/**
		 * @brief Abstract method for getting the cost value given a certain state
		 * @param Eigen::VectorXd state State value
		 * @return double Return the cost at defined state
		 */
		virtual double get(Eigen::VectorXd state);

		/**
		 * @brief Abstract method for getting the cost value given a certain node
		 * @param dwl::planning::AdjacencyMap& adjacency_map Adjacency map required for graph-searching algorithms
		 */
		virtual void get(AdjacencyMap& adjacency_map);

		/**
		 * @brief Indicates if it was defined a cost map in this class
		 * @return bool Return true it was defined a cost map
		 */
		bool isCostMap();

		/**
		 * @brief Gets the name of the cost
		 * @return std::string Return the name of the cost
		 */
		std::string getName();


	protected:
		/** @brief Object of the PlaneGrid class for defining the grid routines */
		environment::PlaneGrid gridmap_;

		/** @brief Name of the cost */
		std::string name_;

		/** @brief Indicates if it is a cost map */
		bool is_cost_map_;

};

} //@namespace planning

} //@namespace dwl


inline std::string dwl::planning::Cost::getName()
{
	return name_;
}


#endif
