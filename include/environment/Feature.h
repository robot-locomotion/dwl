#ifndef DWL_Feature_H
#define DWL_Feature_H

#include <Eigen/Dense>
#include <string>


namespace dwl
{

namespace environment
{


struct Terrain
{
	Eigen::Vector3d position;
	Eigen::Vector3d surface_normal;
	double curvature;
};

class Feature
{
	public:
		Feature();
		virtual ~Feature();

		virtual void computeReward(double& reward_value, Terrain terrain_info) = 0;

		std::string getName();

	protected:
		std::string name_;
		std::vector<double> reward_gridmap_;//TODO: to think in reward_gridmap data

}; //@class Feature


} //@namespace environment

} //@namespace dwl


#endif
