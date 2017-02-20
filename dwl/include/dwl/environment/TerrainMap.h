#ifndef DWL__ENVIRONMENT__TERRAIN_MAP__H
#define DWL__ENVIRONMENT__TERRAIN_MAP__H

#include <dwl/environment/SpaceDiscretization.h>
#include <dwl/utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class TerrainMap
 * @brief Class for defining the terrain information
 */
class TerrainMap
{
	public:
		/** @brief Constructor function */
		TerrainMap();

		/** @brief Destructor function */
		~TerrainMap();

		/** @brief Reset the terrain map */
		void reset();

		/** @brief Sets the terrain data map */
		void setTerrainMap(const TerrainData& terrain_map);
		void setTerrainMap(const TerrainDataMap& map);

		/**
		 * @brief Sets the obstacle map
		 * @param const std::vector<Cell>& Obstacle map
		 */
		void setObstacleMap(const std::vector<Cell>& obstacle_map);

		/**
		 * @brief Sets the values of the terrain cell
		 * @param TerrainCell& Values of the cell
		 * @param double Terrain value of the cell
		 * @param const Terrain& Information of the terrain in the specific cell
		 */
		void setTerrainCell(TerrainCell& cell,
							double cost,
							const Terrain& terrain_info);

		/**
		 * @brief Adds a cell to the terrain map
		 * @param const TerrainCell& Cell values for adding to the terrain map
		 */
		void addCellToTerrainMap(const TerrainCell& cell);

		/**
		 * @brief Removes the cell to the terrain map
		 * @param const Vertex& Cell vertex for removing to the terrain map
		 */
		void removeCellToTerrainMap(const Vertex& cell_vertex);

		/**
		 * @brief Adds a cell to the height map
		 * @param const Vertex& Cell vertex for adding to the height map
		 * @param double Height value
		 */
		void addCellToTerrainHeightMap(const Vertex& cell_vertex,
									   double height);

		/**
		 * @brief Removes a cell to the height map
		 * @param Vertex Cell vertex for removing to the height map
		 */
		void removeCellToTerrainHeightMap(const Vertex& cell_vertex);

		/**
		 * @brief Gets the environment resolution of the terrain map
		 * @param bool Indicates if the key represents a plane or a height
		 * @return The resolution of the gridmap or height
		 */
		double getResolution(bool plane);

		/** @brief Gets the obstacle resolution */
		double getObstacleResolution();

		/**
		 * @brief Sets the resolution of the environment discretization
		 * @param double Resolution of the environment
		 * @param bool Indicates if the key represents a plane or a height
		 */
		void setResolution(double resolution,
						   bool plane);

		/**
		 * @brief Sets the obstacle resolution of the plane or height
		 * @param double Resolution value
		 * @param bool Indicates if the key represents a plane or a height
		 */
		void setObstacleResolution(double resolution,
								   bool plane);

		/**
		 * @brief Sets the state resolution of the plane or height
		 * @param double Position resolution value
		 * @param double Angular resolution value
		 */
		void setStateResolution(double position_resolution,
								double angular_resolution);

		/** @brief Gets the terrain map */
		const TerrainDataMap& getTerrainDataMap() const;

		/** @brief Gets the terrain heightmap */
		const HeightMap& getTerrainHeightMap() const;

		/** @brief Gets the obstacle-map (using vertex id) */
		const ObstacleMap& getObstacleMap() const;

		/**
		 * @brief Gets the terrain data value give a vertex or 2d position
		 * @return The cell data
		 */
		const TerrainCell& getTerrainData(const Vertex& vertex) const;
		const TerrainCell& getTerrainData(const Eigen::Vector2d& position) const;

		/**
		 * @brief Gets the terrain height value give a vertex or 2d position
		 * @return The height value
		 */
		double getTerrainHeight(const Vertex& vertex) const;
		double getTerrainHeight(const Eigen::Vector2d& position) const;

		/**
		 * @brief Gets the terrain reward value give a vertex or 2d position
		 * @return The cost value
		 */
		const Weight& getTerrainCost(const Vertex& vertex) const;
		const Weight& getTerrainCost(const Eigen::Vector2d& position) const;

		/**
		 * @brief Gets the terrain normal value give a vertex or 2d position
		 * @return The cost value
		 */
		const Eigen::Vector3d& getTerrainNormal(const Vertex& vertex) const;
		const Eigen::Vector3d& getTerrainNormal(const Eigen::Vector2d& position) const;

		/**
		 * @brief Gets the terrain discrete model of the space according
		 * the resolution of the terrain
		 * @return The discrete space model
		 */
		const SpaceDiscretization& getTerrainSpaceModel() const;

		/**
		 * @brief Gets the obstacle discrete model of the space according
		 * the resolution of the obstacle map
		 * @return The discrete space model
		 */
		const SpaceDiscretization& getObstacleSpaceModel() const;

		/**
		 * @brief Gets the average cost of the terrain
		 * @return The average cost of the terrain
		 */
		double getAverageCostOfTerrain();

		/**
		 * @brief Indicates if it was defined terrain information
		 * @return True if it was defined terrain cost information, otherwise false
		 */
		bool isTerrainInformation();

		/**
		 * @brief Indicates if it was defined terrain obstacle information
		 * @return True if it was defined terrain obstacle information, otherwise false
		 */
		bool isObstacleInformation();


	protected:
		/** @brief Object of the SpaceDiscretization class for defining the
		 *  grid routines */
		SpaceDiscretization space_discretization_;

		/**
		 * @brief Object of the SpaceDiscretization class for defining the
		 * conversion routines for the terrain obstacle-map */
		environment::SpaceDiscretization obstacle_discretization_;

		/** @brief Terrain values mapped using vertex id */
		TerrainDataMap terrain_map_;

		/** @brief Terrain height map */
		HeightMap terrain_heightmap_;

		/** @brief Gathers the obstacles that are mapped using the vertex id */
		ObstacleMap obstaclemap_;

		/** @brief Default values of the cell, e.g. for unperceived cells */
		TerrainCell default_cell_;

		/** @brief Average terrain cost which is used for unknown areas */
		double average_cost_;

		/** @brief Maximum cost value */
		double max_cost_;

		/** @brief Minimum height of the terrain */
		double min_height_;

		/** @brief Indicates if it was defined terrain information */
		bool terrain_information_;

		/** @brief Indicates if it was defined obstacle information */
		bool obstacle_information_;

		/** @brief Obstacle resolution of the environment */
		double obstacle_resolution_;
};

} //@namespace environment
} //@namespace dwl

#endif
