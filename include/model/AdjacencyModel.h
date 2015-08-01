#ifndef DWL__ENVIRONMENT__ADJACENCY_MODEL__H
#define DWL__ENVIRONMENT__ADJACENCY_MODEL__H

#include <environment/EnvironmentInformation.h>
#include <environment/Feature.h>
#include <robot/Robot.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class AdjacencyModel
 * @brief Abstract class for building an adjacency map which necessary for making the tree
 * exploration with search-tree solver
 */
class AdjacencyModel
{
	public:
		/** @brief Constructor function */
		AdjacencyModel();

		/** @brief Destructor function */
		virtual ~AdjacencyModel();

		/**
		 * @brief Defines the settings of all components within AdjacencyModel class
		 * @param robot::Robot* The robot defines all the properties of the robot
		 * @param environment::EnvironmentInformation* Pointer to object that defines the environment
		 */
		void reset(robot::Robot* robot,
				   environment::EnvironmentInformation* environment);

		/**
		 * @brief Abstract method that computes the whole adjacency map, which is required by some
		 * algorithms such as Dijkstrap
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
		 * @brief Gets the closest start and goal vertex if it is not belong to the terrain information
		 * @param Vertex& The closest vertex to the start
		 * @param Vertex& The closest vertex to the goal
		 * @param Vertex Start vertex
		 * @param Vertex Goal vertex
		 */
		void getTheClosestStartAndGoalVertex(Vertex& closest_source,
											 Vertex& closest_target,
											 Vertex source,
											 Vertex target);

		/**
		 * @brief Gets the closest vertex to a certain vertex
		 * @param Vertex& The closest vertex
		 * @param Vertex Current vertex
		 */
		void getTheClosestVertex(Vertex& closest_vertex,
								 Vertex vertex);

		/**
		 * @brief Estimates the heuristic cost from a source to a target vertex
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 */
		virtual double heuristicCost(Vertex source,
									 Vertex target);

		/**
		 * @brief Indicates if it is reached the goal
		 * @param Vertex Goal vertex
		 * @param Vertex Current vertex
		 * @return True if it is reached the goal and false otherwise
		 */
		bool isReachedGoal(Vertex target,
						   Vertex current);

		/**
		 * @brief Indicates if the free of obstacle
		 * @param Vertex State vertex
		 * @param TypeOfState State representation
		 * @param bool Indicates it is desired to use the body space definition
		 * @return True if it is free of obstacle, and false otherwise
		 */
		virtual bool isFreeOfObstacle(Vertex state_vertex,
									  TypeOfState state_representation,
									  bool body = false);

		/**
		 * @brief Adds a feature for computing the associated body cost
		 * @param environment::Feature* The pointer of the feature to add it
		 */
		void addFeature(environment::Feature* feature);

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

		/** @brief Pointer to robot properties */
		robot::Robot* robot_;

		/** @brief Pointer of the EnvironmentInformation object which describes the environment */
		environment::EnvironmentInformation* environment_;

		/** @brief Vector of pointers to the Feature class */
		std::vector<environment::Feature*> features_;

		/** @brief Indicates if it is a lattice-based graph */
		bool is_lattice_;

		/** @brief Indicates if it was added a feature */
		bool is_added_feature_;

		/** @brief Uncertainty factor which is applied in unperceived environment */
		double uncertainty_factor_; // For unknown (non-perceive) areas
};

} //@namespace model
} //@namespace dwl

#endif
