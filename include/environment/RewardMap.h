#ifndef DWL_RewardMap_H
#define DWL_RewardMap_H

#include <environment/Feature.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{


struct Pose
{
	Eigen::Vector3d position;
	Eigen::Vector4d orientation;
};

struct Modeler
{
	octomap::OcTree* octomap;
	//TODO: other modeler like HeightMap
};

struct SearchArea
{
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double grid_size;
};

class RewardMap
{
	public:
		RewardMap();
		virtual ~RewardMap();

		void addFeature(Feature* feature);

		void removeFeature(std::string feature_name);

		virtual void compute(Modeler model) = 0;

		void setSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_size);




		virtual std::vector<Pose> getNormals() = 0;


	protected:
		std::vector<Feature*> features_;
		SearchArea search_area_;
		bool is_added_feature_;

		pthread_mutex_t environment_lock_;




};


} //@namespace environment

} //@namespace dwl


#endif
