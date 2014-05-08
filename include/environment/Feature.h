#ifndef DWL_Feature_H
#define DWL_Feature_H

#include <Eigen/Dense>
#include <vector>
#include <string>


namespace dwl
{

namespace environment
{


struct Cell
{
	Eigen::Vector3d position;
	//int key;
	double reward;
	int policy;
};

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

		void addCellToRewardMap(double reward, Terrain terrain_info);

		std::string getName();


	protected:
		std::string name_;
		std::vector<Cell> reward_gridmap_;

}; //@class Feature


} //@namespace environment

} //@namespace dwl


#endif
