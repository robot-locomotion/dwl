#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace terrain_server
{

class DefaultFlatTerrain
{
	public:
		DefaultFlatTerrain();
		~DefaultFlatTerrain();

		void setFlatTerrain();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Flat terrain point cloud publisher */
		ros::Publisher flat_terrain_pub_;

		/** @brief Default flat terrain service */
//		ros::ServiceServer flat_terrain_srv_;

		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

};

} //@namespace terrain_server
