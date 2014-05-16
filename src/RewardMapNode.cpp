#ifndef DWL_RewardMapServer_H
#define DWL_RewardMapServer_H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/math/Utils.h>

#include <environment/RewardOctoMap.h>
#include <environment/SlopeFeature.h>

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <reward_map_server/RewardMap.h>
#include <reward_map_server/RewardCell.h>

#include <pthread.h>



class RewardMapServer
{
	public:
		RewardMapServer();
		~RewardMapServer();

		void addFeacture() {}
		void compute() {}

		void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
		void publishRewardMap();

	private:
		ros::NodeHandle node_;
		dwl::environment::RewardMap* reward_map_;
		ros::Publisher reward_pub_;
		//ros::Publisher normal_pub_;
		ros::Subscriber octomap_sub_;
		reward_map_server::RewardMap reward_map_msg_;



		//geometry_msgs::PoseArray normals_;
		//pthread_mutex_t reward_lock_;
		Eigen::Vector3d origin_vector_;


};

#endif



RewardMapServer::RewardMapServer()
{
	origin_vector_ = Eigen::Vector3d::Zero();
	//normals_.poses.resize(2500);
	origin_vector_(0) = 1;

	reward_map_ = new dwl::environment::RewardOctoMap();

	// High resolution
	reward_map_->addSearchArea(0.5, 3.0, -0.75, 0.75, -0.77, -0.4, 0.04);
	// Low resolution
	reward_map_->addSearchArea(3.0, 4.0, -1.0, 1.0, -0.77, -0.4, 0.16);

	//reward_map_->addSearchArea(2.9, 3.0, -0.05, 0.05, -0.77, -0.4, 0.04);
	//reward_map_->addSearchArea(3.2, 3.5, -0.2, 0.2, -0.77, -0.4, 0.16);

	dwl::environment::Feature* slope_ptr = new dwl::environment::SlopeFeature();
	reward_map_->addFeature(slope_ptr);

	octomap_sub_ = node_.subscribe("octomap_full", 1, &RewardMapServer::octomapCallback, this);
	reward_pub_ = node_.advertise<reward_map_server::RewardMap>("reward_map", 1);

	reward_map_msg_.header.frame_id = "base_footprint";

	//normal_pub_ = node_.advertise<geometry_msgs::PoseArray>("normal", 1);
	//normals_.header.frame_id = "base_footprint";
}


RewardMapServer::~RewardMapServer()
{
	delete reward_map_;
	octomap_sub_.shutdown();
}


void RewardMapServer::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	// creating octree
	octomap::OcTree* octomap = NULL;
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

	if (tree) {
		octomap = dynamic_cast<octomap::OcTree*>(tree);
	}

	if (!octomap) {
		ROS_WARN("Failed to create octree structure");
		return;
	}

	dwl::environment::Modeler model;
	model.octomap = octomap;

	Eigen::Vector2d robot_position = Eigen::Vector2d::Zero();
	robot_position(0) = 0;

	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	reward_map_->compute(model, robot_position);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
}


void RewardMapServer::publishRewardMap()
{
	reward_map_msg_.header.stamp = ros::Time::now();

	std::vector<dwl::environment::Cell> reward_gridmap;
	reward_map_->getRewardMap().swap(reward_gridmap);
	reward_map_server::RewardCell cell;
	for (int i = 0; i < reward_gridmap.size(); i++) {
		cell.key_x = reward_gridmap[i].cell_key.grid_id.key[0];
		cell.key_y = reward_gridmap[i].cell_key.grid_id.key[1];
		cell.key_z = reward_gridmap[i].cell_key.height_id;
		cell.reward = reward_gridmap[i].reward;
		cell.cell_size = reward_gridmap[i].size;

		reward_map_msg_.cell.push_back(cell);
	}
	reward_pub_.publish(reward_map_msg_);
	reward_map_msg_.cell.clear();

	/*
	std::vector<dwl::environment::Pose> poses;
	reward_map_->getNormals().swap(poses);
	normals_.header.stamp = ros::Time::now();
	for (int i = 0; i < poses.size(); i++) {
		normals_.poses[i].position.x = poses[i].position(0);
		normals_.poses[i].position.y = poses[i].position(1);
		normals_.poses[i].position.z = poses[i].position(2);
		normals_.poses[i].orientation.x = poses[i].orientation(0);
		normals_.poses[i].orientation.y = poses[i].orientation(1);
		normals_.poses[i].orientation.z = poses[i].orientation(2);
		normals_.poses[i].orientation.w = poses[i].orientation(3);
	}

	normal_pub_.publish(normals_);*/
}







int main(int argc, char **argv)
{
	ros::init(argc, argv, "dwl_reward_map_node");


	RewardMapServer octomap_modeler;
	ros::spinOnce();


	try {
		ros::Rate loop_rate(10);
		while(ros::ok()) {
			octomap_modeler.publishRewardMap();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("octomap_server exception: %s", e.what());
		return -1;
	}

	return 0;
}


