#ifndef DWL_EnvironmentInformation_H
#define DWL_EnvironmentInformation_H

#include <environment/SpaceDiscretization.h>
#include <utils/utils.h>

namespace dwl
{

namespace environment
{

/**
 * @class EnvironmentInformation
 * @brief Class for defining the environment information
 */
class EnvironmentInformation
{
	public:
		/** @brief Constructor function */
		EnvironmentInformation();

		/** @brief Destructor function */
		~EnvironmentInformation();

		/**
		 * @brief Sets the terrain cost-map and height-map
		 * @param std::vector<dwl::RewardCell> reward_map Reward map
		 */
		void setEnvironmentInformation(std::vector<RewardCell> reward_map);

		/**
		 * @brief Sets the obstacle map
		 * @param std::vector<dwl::Cell> obstacle_map Obstacle map
		 */
		void setEnvironmentInformation(std::vector<Cell> obstacle_map);

		/**
		 * @brief Sets the terrain resolution (cost and height map) of the plane or height
		 * @param double resolution Resolution value
		 * @param bool plane Indicates if the key represents a plane or a height
		 */
		void setTerrainResolution(double resolution, bool plane);

		/**
		 * @brief Sets the obstacle resolution of the plane or height
		 * @param double resolution Resolution value
		 * @param bool plane Indicates if the key represents a plane or a height
		 */
		void setObstacleResolution(double resolution, bool plane);

		/**
		 * @brief Sets the state resolution of the plane or height
		 * @param double position_resolution Position resolution value
		 * @param double angular_resolution Angular resolution value
		 */
		void setStateResolution(double position_resolution, double angular_resolution);

		/**
		 * @brief Gets the terrain cost-map (using vertex id)
		 * @param dwl::CostMap& costmap Cost map of the terrain
		 */
		void getTerrainCostMap(CostMap& costmap);

		/**
		 * @brief Gets the terrain height-map (using vertex id)
		 * @param dwl::HeightMap& heightmap Height map of the terrain
		 */
		void getTerrainHeightMap(HeightMap& heightmap);

		/**
		 * @brief Gets the obstacle-map (using vertex id)
		 * @param dwl::ObstacleMap& obstaclemap Obstacle map of the terrain
		 */
		void getObstacleMap(ObstacleMap& obstaclemap);

		/** @brief Gets the terrain resolution (cost and height map) */
		double getTerrainResolution();

		/** @brief Gets the obstacle resolution */
		double getObstacleResolution();

		/**
		 * @brief Gets the average cost of the terrain
		 * @return double Returns the average cost of the terrain
		 */
		double getAverageCostOfTerrain();

		/**
		 * @brief Gets the cost discretizated model of the space according the resolution of the terrain
		 * @return dwl::environment::SpaceDiscretization& Returns the discretized space model
		 */
		SpaceDiscretization& getTerrainSpaceModel();

		/**
		 * @brief Gets the cost discretizated model of the space according the resolution of the reward map
		 * @return dwl::environment::SpaceDiscretization& Returns the discretized space model
		 */
		SpaceDiscretization& getObstacleSpaceModel();

		/**
		 * @brief Indicates if it was defined terrain information
		 * @return Returns true if it was defined terrain cost information, otherwise false
		 */
		bool isTerrainInformation();

		/**
		 * @brief Indicates if it was defined terrain obstacle information
		 * @return Returns true if it was defined terrain obstacle information, otherwise false
		 */
		bool isObstacleInformation();

	private:
		/** @brief Object of the SpaceDiscretization class for defining the conversion routines for the terrain cost-map */
		environment::SpaceDiscretization terrain_space_discretization_;

		/** @brief Object of the SpaceDiscretization class for defining the conversion routines for the terrain obstacle-map */
		environment::SpaceDiscretization obstacle_space_discretization_;

		/** @brief Gathers the terrain cost values that are mapped using the vertex id */
		CostMap terrain_cost_map_;

		/** @brief Gathers the obstacles that are mapped using the vertex id */
		ObstacleMap obstacle_map_;

		/** @brief Gathers the height values that are mapperd using the vertex id */
		HeightMap terrain_height_map_;

		/** @brief Gathers the body cost values that are mapped using the vertex id */
		CostMap body_cost_map_;

		/** @brief Average terrain cost which is used for unknown areas */
		double average_terrain_cost_;

		/** @brief Indicates if it was defined terrain information */
		bool terrain_information_;

		/** @brief Indicates if it was defined obstacle information */
		bool obstacle_information_;

		/** @brief Terrain resolution (cost and height map) of the environment */
		double terrain_resolution_;

		/** @brief Obstacle resolution of the environment */
		double obstacle_resolution_;
};

} //@namespace environment
} //@namespace dwl

#endif
