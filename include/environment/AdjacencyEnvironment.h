#ifndef DWL_AdjacencyEnvironment_H
#define DWL_AdjacencyEnvironment_H

#include <environment/PlaneGrid.h>
#include <robot/Robot.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class AdjacencyEnvironment
 * @brief Abstract class for building adjacency map of the environment
 */
class AdjacencyEnvironment
{
	public:
		/** @brief Constructor function */
		AdjacencyEnvironment();

		/** @brief Destructor function */
		virtual ~AdjacencyEnvironment();

		/**
		 * @brief Sets the terrain cost map from the reward map information
		 * @param std::vector<dwl::Cell> reward_map Reward map
		 */
		void setEnvironmentInformation(std::vector<Cell> reward_map);

		/**
		 * @brief Abstract the sets the resolution of the height or grid
		 * @param double resolution Resolution value
		 * @param bool gridmap Defines the resolution to set, i.e. gridmap (true) or height (false)
		 */
		void setResolution(double resolution, bool gridmap); //TODO check that I'm setting this resolution from locomotion class

		/**
		 * @brief Abstract method that computes the whole adjancecy map, which is required by some algorithms such as Dijkstrap
		 * @param dwl::AdjacencyMap& adjacency_map Adjacency map
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @param Eigen::Vector3d position 2D position and orientation
		 */
		virtual void computeAdjacencyMap(AdjacencyMap& adjacency_map, Vertex source, Vertex target, double orientation);//Eigen::Vector3d position);

		/**
		 * @brief Abstract method that gets the successors of a certain vertex
		 * @param std::list<Edge>& successors The successors of a certain vertex
		 * @param dwl::Vertex Current vertex
		 * @param double orientation Current orientation
		 */
		virtual void getSuccessors(std::list<Edge>& successors, Vertex vertex, double orientation);

		/**
		 * @brief Gets the closest start and goal vertex if it is not belong to the terrain information
		 * @param dwl::Vertex& closest_source The closest vertex to the start
		 * @param dwl::Vertex& closest_target The closest vertex to the goal
		 * @param dwl::Vertex source Start vertex
		 * @param dwl::Vertex target Goal vertex
		 */
		void getTheClosestStartAndGoalVertex(Vertex& closest_source, Vertex& closest_target, Vertex source, Vertex target);

		/**
		 * @brief Gets the closest vertex to a certain vertex
		 * @param dwl::Vertex& closest_vertex The closest vertex
		 * @param dwl::Vertex Current vertex
		 */
		void getTheClosestVertex(Vertex& closest_vertex, Vertex vertex);

		/**
		 * @brief Estimates the heuristic cost from a source to a target vertex
		 * @param Vertex source Source vertex
		 * @param Vertex target Target vertex
		 */
		virtual double heuristicCostEstimate(Vertex source, Vertex target);

		/**
		 * @brief Indicates if it is reached the goal
		 * @param dwl::Vertex target Goal vertex
		 * @param dwl::Vertex current Current vertex
		 * @return bool Return true if it is reache the goal and false otherwise
		 */
		bool isReachedGoal(Vertex target, Vertex current);

		/**
		 * @brief Indicates if it is a lattice representation of the environment
		 * @return bool Return true if it is a lattice representation and false otherwise
		 */
		bool isLatticeRepresentation();

		/**
		 * @brief Gets the 2d position of the current vertex
		 * @param dwl::Vertex vertex Current vertex
		 * @return Eigen::Vector2d Return the 2d position
		 */
		Eigen::Vector2d getPosition(Vertex vertex);

		Vertex getVertex(Pose pose);//TODO

		/**
		 * @brief Gets the name of the adjacency model
		 * @return std::string Return the name of the adjacency model
		 */
		std::string getName();


	protected:
		/** @brief Name of the adjacency model */
		std::string name_;

		/** @brief Object of the PlaneGrid class for defining the grid routines */
		environment::PlaneGrid gridmap_;

		/** @brief Gathers the cost values that are mapped using the vertex id */
		CostMap terrain_cost_map_;

		/** @brief Odject of the Robot class for defining robot properties */
		robot::Robot robot_;

		/** @brief Average cost which is used for unknown areas */
		double average_cost_;

		bool is_there_terrain_information_;

		bool is_lattice_;


	private:

};


} //@namespace environment

} //@namespace dwl


#endif
