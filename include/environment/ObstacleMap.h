#ifndef DWL_ObstacleMap_H
#define DWL_ObstacleMap_H

#include <environment/PlaneGrid.h>
#include <utils/utils.h>

#include <octomap/octomap.h>


namespace dwl
{

namespace environment
{

/*struct Cell
{
	Key key;
	double plane_size;
	double height_size;
};*/

/**
 * @class ObstacleMap
 * @brief Abstract class for computing the obstacle map of the terrain
 */
class ObstacleMap
{
	public:
		ObstacleMap();
		~ObstacleMap();

		/**
		 * @brief Computes the obstacle map according the robot position and model of the terrain
		 * @param octomap::OcTree* octomap Octomap model of the environment
		 * @param Eigen::Vector4d robot_state The position of the robot and the yaw angle
		 */
		void compute(octomap::OcTree* octomap, Eigen::Vector4d robot_state) = 0;

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
};

} //@namespace environment
} //@namespace dwl

#endif
