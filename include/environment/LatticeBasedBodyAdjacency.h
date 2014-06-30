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
		 * @param dwl::Vertex vertex Current vertex
		 * @param double orientation Current orientation
		 */
		void getSuccessors(std::list<Edge>& successors, Vertex vertex, double orientation);


	private:
		/**
		 * @brief Searchs the neighbors of a current vertex
		 * @param std::vector<Vertex>& neighbors The set of neighbors
		 * @param dwl::Vertex vertex_id Current vertex
		 */
		void searchNeighbors(std::vector<Vertex>& neighbors, Vertex vertex_id);

		/**
		 * @brief Computes the body cost of a current vertex
		 * @param dwl::Vertex vertex Current vertex
		 * @param double orientation Current orientation
		 */
		void computeBodyCost(double& cost, Vertex vertex, double orientation);

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

		/** @brief Uncertainty factor which is applicated in un-perceived environment */
		double uncertainty_factor_; // For unknown (non-perceive) areas
};

} //@namespace environmet
} //@namespace dwl

#endif
