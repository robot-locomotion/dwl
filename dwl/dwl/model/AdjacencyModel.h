#ifndef DWL__MODEL__ADJACENCY_MODEL__H
#define DWL__MODEL__ADJACENCY_MODEL__H

#include <dwl/utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @brief The AdjacencyModel class
 * It models an adjacency map needed for search-tree solvers of dwl. This class
 * allows us to comput the adjacency map, get the successors, compute the
 * heuristic cost, evaluated obstacles and when the target is reached.
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 */
class AdjacencyModel
{
	public:
		/** @brief Constructor function */
		AdjacencyModel();

		/** @brief Destructor function */
		virtual ~AdjacencyModel();

		/**
		 * @brief Abstract method that computes the whole adjacency map, which
		 * is required by some algorithms such as Dijkstrap
		 * @param AdjacencyMap& Adjacency map
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 */
		virtual void computeAdjacencyMap(AdjacencyMap& adjacency_map,
										 Vertex source,
										 Vertex target);

		/**
		 * @brief Abstract method that gets the successors of a certain vertex
		 * @param std::list<Edge>& The successors of a certain vertex
		 * @param Vertex Current state vertex
		 */
		virtual void getSuccessors(std::list<Edge>& successors,
								   Vertex state_vertex) = 0;

		/**
		 * @brief Estimates the heuristic cost from a source to a target vertex
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 */
		virtual double heuristicCost(Vertex source,
									 Vertex target) = 0;

		/**
		 * @brief Indicates if it is reached the goal
		 * @param Vertex Goal vertex
		 * @param Vertex Current vertex
		 * @return True if it is reached the goal and false otherwise
		 */
		bool virtual isReachedGoal(Vertex target,
								   Vertex current) = 0;

		/**
		 * @brief Indicates if the free of obstacle
		 * @param Vertex State vertex
		 * @param TypeOfState State representation
		 * @param bool Indicates it is desired to use the body space definition
		 * @return True if it is free of obstacle, and false otherwise
		 */
		virtual bool isFreeOfObstacle(Vertex state_vertex,
									  TypeOfState state_representation,
									  bool body = false) = 0;

		/**
		 * @brief Indicates if it is a lattice representation of the environment
		 * @return True if it is a lattice representation and false otherwise
		 */
		bool isLatticeRepresentation();

		/**
		 * @brief Gets the name of the adjacency model
		 * @return The name of the adjacency model
		 */
		std::string getName();


	protected:
		/** @brief Name of the adjacency model */
		std::string name_;

		/** @brief Indicates if it is a lattice-based graph */
		bool is_lattice_;
};

} //@namespace model
} //@namespace dwl

#endif
