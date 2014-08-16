#ifndef DWL_LatticeBasedBodyAdjacency_H
#define DWL_LatticeBasedBodyAdjacency_H

#include <environment/AdjacencyEnvironment.h>
#include <behavior/MotorPrimitives.h>


namespace dwl
{

namespace environment
{

/**
 * @class LatticeBasedBodyAdjacency
 * @brief Class for building a lattice-based adjacency map of the environment
 */
class LatticeBasedBodyAdjacency : public AdjacencyEnvironment
{
	public:
		/** @brief Constructor function */
		LatticeBasedBodyAdjacency();

		/** @brief Destructor function */
		~LatticeBasedBodyAdjacency();

		/**
		 * @brief Gets the successors of the current vertex
		 * @param std::list<Edge>& successors List of successors
		 * @param dwl::Vertex state_vertex Current state vertex
		 */
		void getSuccessors(std::list<Edge>& successors, Vertex state_vertex);


	private:
		/**
		 * @brief Searchs the neighbors of a current vertex
		 * @param std::vector<Vertex>& neighbors The set of neighbors
		 * @param dwl::Vertex vertex_id Current vertex
		 */
		void searchNeighbors(std::vector<Vertex>& neighbors, Vertex vertex_id);

		/**
		 * @brief Computes the body cost of a current vertex
		 * @param dwl::Vertex state_vertex Current state vertex
		 */
		void computeBodyCost(double& cost, Vertex state_vertex);

		/**
		 * @brief Indicates if the free of obstacle
		 * @param dwl::Vertex state_vertex State vertex
		 * @param dwl::TypeOfState state_representation State representation
		 * @param bool body Indicates it is desired to use the body space definition
		 * @return bool Returns true if it is free of obstacle, and false otherwise
		 */
		bool isFreeOfObstacle(Vertex state_vertex, TypeOfState state_representation, bool body=false);

		/**
		 * @brief Indicates if it is requested a stance adjacency
		 * @return Returns true it is requested a stance adjacency (body cost), false otherwise
		 */
		bool isStanceAdjacency();

		/** @brief Pointer to the motor primitives */
		behavior::MotorPrimitives* behavior_;

		/** @brief Indicates it was requested a stance or terrain adjacency */
		bool is_stance_adjacency_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		/** @brief Number of top reward for computing the stance cost */
		int number_top_reward_;
};

} //@namespace environmet
} //@namespace dwl

#endif
