#ifndef DWL_RewardMap_H
#define DWL_RewardMap_H

#include <environment/PlaneGrid.h>
#include <environment/Feature.h>
#include <utils/utils.h>

#include <octomap/octomap.h>


namespace dwl
{

namespace environment
{

/**
 * @class RewardMap
 * @brief Abstract class for computing the reward map of the terrain
 */
class RewardMap
{
	public:
		/** @brief Constructor function */
		RewardMap();

		/** @brief Destructor function */
		virtual ~RewardMap();

		/**
		 * @brief Adds a feature of the reward map
		 * @param dwl::environment::Feature* feature the pointer of the feature to add
		 */
		void addFeature(Feature* feature);

		/**
		 * @brief Removes a feature of the reward map
		 * @param std::string feature_name the name of the feature to remove
		 */
		void removeFeature(std::string feature_name);

		/**
		 * @brief Abstract method for computing the reward map according the robot position and model of the terrain
		 * @param dwl::TerrainModel model The model of the environment
		 * @param Eigen::Vector4d robot_state The position of the robot and the yaw angle
		 */
		virtual void compute(TerrainModel model, Eigen::Vector4d robot_state) = 0;

		/**
		 * @brief Removes reward values outside the interest region
		 * @param Eigen::Vector3d robot_state State of the robot, i.e. 3D position and yaw orientation
		 */
		void removeRewardOutsideInterestRegion(Eigen::Vector3d robot_state);

		/**
		 * @brief Sets a interest region
		 * @param double radius_x Radius along the x-axis
		 * @param double radius_y Radius along the x-axis
		 */
		void setInterestRegion(double radius_x, double radius_y);

		/**
		 * @brief Gets the properties of the cell
		 * @param dwl::Cell& cell Values of the cell
		 * @param double reward Reward value of the cell
		 * @param dwl::environment::Terrain terrain_info Information of the terrain in the specific cell
		 */
		void getCell(Cell& cell, double reward, Terrain terrain_info);

		/**
		 * @brief Gets the properties of the cell
		 * @param dwl::Key& key Key of the cell
		 * @param Eigen::Vector3d position Cartesian position of the cell
		 */
		void getCell(Key& key, Eigen::Vector3d position);

		/**
		 * @brief Adds a cell to the reward map
		 * @param dwl::Cell cell Cell values for adding to the reward map
		 */
		void addCellToRewardMap(Cell cell);

		/**
		 * @brief Removes the cel to the reward map
		 * @param Vertex cell_vertex Cell vertex for removing to the reward map
		 */
		void removeCellToRewardMap(Vertex cell_vertex);

		/**
		 * @brief Adds a cell to the height map
		 * @param Vertex vertex Cell vertex for adding to the height map
		 * @param double height Height value
		 */
		void addCellToTerrainHeightMap(Vertex cell_vertex, double height);

		/**
		 * @brief Removes a cell to the height map
		 * @para Vertex cell_vertex Cell vertex for removing to the height map
		 */
		void removeCellToTerrainHeightMap(Vertex cell_vertex);

		/**
		 * @brief Adds a new search area around the current position of the robot
		 * @param double min_x Minimun cartesian position along the x-axis
		 * @param double max_x Maximun cartesian position along the x-axis
		 * @param double min_y Minimun cartesian position along the y-axis
		 * @param double max_x Maximun cartesian position along the y-axis
		 * @param double min_z Minimun cartesian position along the z-axis
		 * @param double max_z Maximun cartesian position along the z-axis
		 * @param double grid_size Resolution of the grid
		 */
		void addSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_size);

		/**
		 * @brief Sets the neighboring area for computing phisycal properties of the terrain
		 * @param int back_neighbors Number of left neighbors
		 * @param int front_neighbors Number of right neighbors
		 * @param int left_neighbors Number of left neighbors
		 * @param int right_neighbors Number of right neighbors
		 * @param int bottom_neighbors Number of bottom neighbors
		 * @param int top_neighbors Number of top neighbors
		 */
		void setNeighboringArea(int back_neighbors, int front_neighbors, int left_neighbors, int right_neighbors, int bottom_neighbors, int top_neighbors);

		/**
		 * @brief Gets the environment resolution of the reward map
		 * @param double Returns the resolution of the gridmap or height
		 * @param bool plane Indicates if the key represents a plane or a height
		 */
		double getResolution(bool plane);

		/**
		 * @brief Sets the resolution of the environment discretization
		 * @ double resolution Resolution of the environment
		 * @param bool plane Indicates if the key represents a plane or a height
		 */
		void setResolution(double resolution, bool plane);

		/**
		 * @brief Gets the reward map
		 * @return std::map<Vertex,dwl::environment::Cell> Returns the cell value per each vertex
		 */
		const std::map<Vertex,Cell>& getRewardMap() const;


	protected:
		/** @brief Object of the SpaceDiscretization class for defining the grid routines */
		SpaceDiscretization space_discretization_;

		/** @brief Vector of pointers to the Feature class */
		std::vector<Feature*> features_;

		/** @brief Reward values mapped using vertex id */
		std::map<Vertex,Cell> reward_gridmap_;

		/** @brief Terrain height map */
		std::map<Vertex, double> terrain_heightmap_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> search_areas_;

		/** @brief Interest area */
		double interest_radius_x_, interest_radius_y_;

		/** @brief Cell size */
		double cell_size_;

		/** @brief Object of the NeighboringArea struct that defines the neighboring area */
		NeighboringArea neighboring_area_;

		/** @brief Indicates if it was added a feature */
		bool is_added_feature_;

		/** @brief Indicates if it was added a search area */
		bool is_added_search_area_;



//		pthread_mutex_t environment_lock_;
};

} //@namespace environment
} //@namespace dwl


#endif
