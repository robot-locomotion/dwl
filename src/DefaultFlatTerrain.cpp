#include <terrain_server/DefaultFlatTerrain.h>
//#include "pcl_ros/point_cloud.h"

namespace terrain_server
{

DefaultFlatTerrain::DefaultFlatTerrain()
{

//	flat_terrain_srv_ = node_.advertiseService("terrain_server/default_flat_terrain", &DefaultFlatTerrain::setFlatTerrain, this);
	flat_terrain_pub_  = node_.advertise<sensor_msgs::PointCloud2>("head_mount_asus_xtion/depth_registered/points", 1);//TODO read topic
}


DefaultFlatTerrain::~DefaultFlatTerrain()
{

}


void DefaultFlatTerrain::setFlatTerrain()
{
	double x_min = -1;
	double y_min = -1;
	double x_max = 1;
	double y_max = 1;
	double resolution = 0.01;
	double z = -0.54;


	// Computing the boundary properties
	double center_x = x_min + (x_max - x_min) / 2;
	double center_y = y_min + (y_max - y_min) / 2;
	double window_x = (x_max - x_min) / 2;
	double window_y = (y_max - y_min) / 2;

	PointCloud pcl_msg;
	pcl_msg.header.frame_id = "odom";
	for (double xi = 0; xi < window_x; xi += resolution) {
		for (int sx = -1; sx <= 1; sx += 2) {
			for (double yi = 0; yi < window_y; yi += resolution) {
				for (int sy = -1; sy <= 1; sy += 2) {
					double x = sx * xi + center_x;
					double y = sy * yi + center_y;
//					pcl_msg.height = pcl_msg.width = 1;
					pcl_msg.push_back(pcl::PointXYZ(x, y, z));
				}
			}
		}
	}

	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg (pcl_msg, cloud);

	flat_terrain_pub_.publish(cloud);
}

} //@namespace terrain_server



int main(int argc, char **argv)
{
	ros::init(argc, argv, "default_flat_terrain");

	terrain_server::DefaultFlatTerrain default_flat_terrain;

	ros::spinOnce();

	try {
		ros::Rate loop_rate(100);
		while(ros::ok()) {
			default_flat_terrain.setFlatTerrain();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("reward_map_server exception: %s", e.what());
		return -1;
	}

	return 0;
}
