#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace terrain_server
{

class DefaultFlatTerrain
{
	public:
		DefaultFlatTerrain(ros::NodeHandle node);
		~DefaultFlatTerrain();

		void setFlatTerrain();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Private ROS node handle */
		ros::NodeHandle private_node_;

		/** @brief Flat terrain point cloud publisher */
		ros::Publisher flat_terrain_pub_;

		/** @brief Default flat terrain service */
//		ros::ServiceServer flat_terrain_srv_;

		double x_min_;// = -0.50;
		double y_min_;// = -0.35;
		double x_max_;// = 0.80;
		double y_max_;// = 0.35;
		double resolution_;// = 0.01;
		double height_;// = -0.54;

		std::string world_frame_;

		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

};

} //@namespace terrain_server
