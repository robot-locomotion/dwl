#ifndef DWL_ObstacleMap_H
#define DWL_ObstacleMap_H

#include <environment/SpaceDiscretization.h>
#include <utils/utils.h>

#include <octomap/octomap.h>


namespace dwl
{

namespace environment
{

/**
 * @class ObstacleMap
 * @brief Abstract class for computing the obstacle map of the terrain
 */
class ObstacleMap
{
	public:
		/** @brief Constructor function */
		ObstacleMap();

		/** @brief Destructor function */
		~ObstacleMap();

		/**
		 * @brief Computes the obstacle map according the robot position and model of the terrain
		 * @param octomap::OcTree* octomap Octomap model of the environment
		 * @param Eigen::Vector4d robot_state The position of the robot and the yaw angle
		 */
		void compute(octomap::OcTree* octomap, Eigen::Vector4d robot_state);

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
		 * @brief Removes obstacles outside the interest region
		 * @param Eigen::Vector3d robot_state State of the robot, i.e. 3D position and yaw orientation
		 */
		void removeObstacleOutsideInterestRegion(Eigen::Vector3d robot_state);

		/**
		 * @brief Adds a cell to the obstacle map
		 * @param dwl::Cell cell Cell values for adding to the reward map
		 */
		void addCellToObstacleMap(Cell cell);

		/**
		 * @brief Sets a interest region
		 * @param double radius_x Radius along the x-axis
		 * @param double radius_y Radius along the x-axis
		 */
		void setInterestRegion(double radius_x, double radius_y);

		/**
		 * @brief Gets the environment resolution of the obstacle map
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
		 * @brief Gets the obstacle map
		 * @return std::map<Vertex,Cell> Returns the cell value per each vertex
		 */
		const std::map<Vertex,Cell>& getObstacleMap() const;


	private:
		/** @brief Object of the SpaceDiscretization class for defining the grid routines */
		SpaceDiscretization space_discretization_;

		/** @brief Reward values mapped using vertex id */
		std::map<Vertex,Cell> obstacle_gridmap_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> search_areas_;

		/** @brief Depth of the octomap model for searching obstacles */
		int depth_;

		/** @brief Indicates if it was added a search area */
		bool is_added_search_area_;

		/** @brief Interest area */
		double interest_radius_x_, interest_radius_y_;

		/** @brief Resolution of the obstacle map server */
		double resolution_;
};

} //@namespace environment
} //@namespace dwl

#endif
