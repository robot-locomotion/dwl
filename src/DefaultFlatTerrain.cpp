#include <terrain_server/DefaultFlatTerrain.h>



namespace terrain_server
{

DefaultFlatTerrain::DefaultFlatTerrain(ros::NodeHandle node) : node_(node), x_min_(0.0), x_max_(0.0),
		y_min_(0.0), y_max_(0.0), resolution_(0.0), height_(0.0), world_frame_("odom")
{
	flat_terrain_pub_  = private_node_.advertise<sensor_msgs::PointCloud2>("topic_output", 1);
	node_.param("world_frame", world_frame_, world_frame_);
	node_.param("x_min", x_min_, x_min_);
	node_.param("x_max", x_max_, x_max_);
	node_.param("y_min", y_min_, y_min_);
	node_.param("y_max", y_max_, y_max_);
	node_.param("height", height_, height_);
	node_.param("resolution", resolution_, resolution_);
}


DefaultFlatTerrain::~DefaultFlatTerrain()
{

}


void DefaultFlatTerrain::setFlatTerrain()
{
	// Computing the boundary properties
	double center_x = x_min_ + (x_max_ - x_min_) / 2;
	double center_y = y_min_ + (y_max_ - y_min_) / 2;
	double window_x = (x_max_ - x_min_) / 2;
	double window_y = (y_max_ - y_min_) / 2;

	PointCloud pcl_msg;
	pcl_msg.header.frame_id = "world";
	for (double xi = 0; xi < window_x; xi += resolution_) {
		for (int sx = -1; sx <= 1; sx += 2) {
			for (double yi = 0; yi < window_y; yi += resolution_) {
				for (int sy = -1; sy <= 1; sy += 2) {
					double x = sx * xi + center_x;
					double y = sy * yi + center_y;
//					pcl_msg.height = pcl_msg.width = 1;
					pcl_msg.push_back(pcl::PointXYZ(x, y, height_));
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

	ros::NodeHandle node("~");
	terrain_server::DefaultFlatTerrain default_flat_terrain(node);

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
