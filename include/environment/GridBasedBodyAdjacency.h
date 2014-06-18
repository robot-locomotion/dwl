#ifndef DWL_GridBasedBodyAdjacency_H
#define DWL_GridBasedBodyAdjacency_H

#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{

class GridBasedBodyAdjacency : public AdjacencyEnvironment
{
	public:
		GridBasedBodyAdjacency();
		~GridBasedBodyAdjacency();

		void computeAdjacencyMap(AdjacencyMap& adjacency_map, Eigen::Vector3d position);

		void getSuccessors(std::list<Edge>& successors, Vertex vertex);


	private:
		/**
		 * @brief Adds the cost to adjacent vertexs
		 * @param AdjacencyMap& adjacency_map Adjacency map
		 * @param Vertex vertex_id Vertex id
		 * @param double cost Cost value
		 */
		void addCostToAdjacentVertexs(AdjacencyMap& adjacency_map, Vertex vertex_id, double cost);

		void searchNeighbours(std::vector<Vertex>& neighbours, Vertex vertex_id);

		bool isStanceAdjacency();

		bool is_stance_adjacency_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		int number_top_reward_;

		double uncertainty_factor_; // For unknown (non-perceive) areas



};

} //@namespace hyq

} //@namespace dwl

#endif
