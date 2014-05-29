#ifndef DWL_RewardMap_H
#define DWL_RewardMap_H

#include <environment/PlaneGrid.h>
#include <environment/Feature.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{

// TODO: Pose struct temporaly, only for testing
struct Pose
{
	Eigen::Vector3d position;
	Eigen::Vector4d orientation;
};

/**
 * @brief Struct that defines the id (key) of a certain cell
 */
struct CellKey
{
	Key grid_id;
	unsigned short int height_id;
};

/**
 * @brief Struct that defines the information of the cell
 */
struct Cell
{
	CellKey cell_key;
	double reward;
	double size;
};

/**
 * @brief Struct that defines the models of the environment
 */
struct Modeler
{
	octomap::OcTree* octomap;
	//TODO: To integrate others modeler like HeightMap
};

/**
 * @brief Struct that defines the search area
 */
struct SearchArea
{
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double grid_resolution;
};

/**
 * @struct NeighboringArea
 * @brief Struct that defines the neighboring area
 */
struct NeighboringArea
{
	int min_x, max_x;
	int min_y, max_y;
	int min_z, max_z;
};

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
		 * @param dwl::environment::Modeler model The model of the environment
		 * @param Eigen::Vector4d robot_state The position of the robot and the yaw angle
		 */
		virtual void compute(Modeler model, Eigen::Vector4d robot_state) = 0;

		/**
		 * @brief Removes reward values outside the interest region
		 * @param Eigen::Vector3d robot_state State of the robot, i.e. 3D position and yaw orientation
		 */
		void removeRewardOutsideInterestRegion(Eigen::Vector3d robot_state);

		/**
		 * @brief Sets a interest region
		 * @param double min_x Minimun cartesian position along the x-axis
		 * @param double max_x Maximun cartesian position along the x-axis
		 * @param double min_y Minimun cartesian position along the y-axis
		 * @param double max_x Maximun cartesian position along the y-axis
		 */
		void setInterestRegion(double min_x, double max_x, double min_y, double max_y);

		/**
		 * @brief Gets the properties of the cell
		 * @param dwl::environment::Cell& cell Values of the cell
		 * @param double reward Reward value of the cell
		 * @param dwl::environment::Terrain terrain_info Information of the terrain in the specific cell
		 */
		void getCell(Cell& cell, double reward, Terrain terrain_info);

		/**
		 * @brief Gets the properties of the cell
		 * @param dwl::environment::CellKey& cell_key Key of the cell
		 * @param Eigen::Vector3d position Cartesian position of the cell
		 */
		void getCell(CellKey& cell_key, Eigen::Vector3d position);

		/**
		 * @brief Adds a cell to the reward map
		 * @param dwl::environment::Cell cell Cell values for adding to the reward map
		 */
		void addCellToRewardMap(Cell cell);

		/**
		 * @brief Removes the cel to the reward map
		 * @param CellKey cell Cell key for removing to the reward map
		 */
		void removeCellToRewardMap(CellKey cell);

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
		 * @brief Gets the gridmap or height resolution of the reward map
		 * @param bool gridmap
		 */
		double getResolution(bool gridmap);


		/**
		 * @brief Sets the resolution of the modeler
		 * @ double resolution Resolution
		 */
		void setModelerResolution(double resolution);

		/**
		 * @brief Gets the reward map
		 * @return std::vector<dwl::environment::Cell> Return the reward value per every cell of the map
		 */
		std::vector<Cell> getRewardMap();



		virtual std::vector<Pose> getNormals() = 0;


	protected:
		/** @brief Object of the PlaneGrid class for defining the grid routines */
		PlaneGrid gridmap_;

		/** @brief Vector of pointers to the Feature class */
		std::vector<Feature*> features_;

		/** @brief Vector of the reward values of the cell */
		std::vector<Cell> reward_gridmap_; //TODO use std::map for ordering the cell using a vertex id (To think in this)

		/** @brief Vector of search areas */
		std::vector<SearchArea> search_areas_;

		/** @brief Interest area */
		SearchArea interest_area_;

		/** @brief Cell size */
		double cell_size_;

		/** @brief Object of the NeighboringArea struct that defines the neighboring area */
		NeighboringArea neighboring_area_;

		/** @brief Indicates if it was added a feature */
		bool is_added_feature_;

		/** @brief Indicates if it was added a search area */
		bool is_added_search_area_;



		pthread_mutex_t environment_lock_;
};


} //@namespace environment

} //@namespace dwl


#endif
