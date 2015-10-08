#ifndef TERRAIN_SERVER__DEFAULT_FLAT_TERRAIN___H
#define TERRAIN_SERVER__DEFAULT_FLAT_TERRAIN___H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


namespace terrain_server
{

/**
 * @class DefaultFlatTerrain
 * @brief Class for building default flat terrain around the robot
 */
class DefaultFlatTerrain
{
	public:
		/** @brief Constructor function */
		DefaultFlatTerrain(ros::NodeHandle node);

		/** @brief Destructor function */
		~DefaultFlatTerrain();

		/** @brief Sets the flat terrain given the desired properties */
		void setFlatTerrain();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Private ROS node handle */
		ros::NodeHandle private_node_;

		/** @brief Flat terrain point cloud publisher */
		ros::Publisher flat_terrain_pub_;

		/** @brief Flat terrain properties */
		double x_min_;// = -0.50;
		double y_min_;// = -0.35;
		double x_max_;// = 0.80;
		double y_max_;// = 0.35;
		double resolution_;// = 0.01;
		double height_;// = -0.54;

		/** @brief World frame name */
		std::string world_frame_;

		/** Point cloud data that defines the flat terrain */
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
};

} //@namespace terrain_server

#endif
