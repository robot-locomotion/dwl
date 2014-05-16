#ifndef DWL_RewardMap_H
#define DWL_RewardMap_H

#include <environment/PlaneGrid.h>
#include <environment/Feature.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{

// TODO: Pose struct temporaly, only for testing
struct Pose
{
	Eigen::Vector3d position;
	Eigen::Vector4d orientation;
};

struct CellKey
{
	Key grid_id;
	unsigned short int height_id;
};

struct Cell
{
	CellKey cell_key;
	double reward;
	double size;
};

struct Modeler
{
	octomap::OcTree* octomap;
	//TODO: To integrate others modeler like HeightMap
};

struct SearchArea
{
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double grid_resolution;
};

struct NeighboringArea
{
	int min_x, max_x;
	int min_y, max_y;
	int min_z, max_z;
};

class RewardMap
{
	public:
		RewardMap();
		virtual ~RewardMap();

		void addFeature(Feature* feature);
		void removeFeature(std::string feature_name);


		virtual void compute(Modeler model, Eigen::Vector2d robot_position) = 0;

		void getCell(Cell &cell, double reward, Terrain terrain_info);
		void getCell(CellKey& cell_key, Eigen::Vector3d position);

		void addCellToRewardMap(Cell cell);
		void removeCellToRewardMap(CellKey cell);


		void addSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_size);
		void setNeighboringArea(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);

		std::vector<Cell> getRewardMap();

		virtual std::vector<Pose> getNormals() = 0;


	protected:
		PlaneGrid gridmap_;
		std::vector<Feature*> features_;
		std::vector<Cell> reward_gridmap_;
		std::vector<SearchArea> search_areas_;
		double cell_size_;
		NeighboringArea neighboring_area_;
		bool is_added_feature_;
		bool is_added_search_area_;

		pthread_mutex_t environment_lock_;
};


} //@namespace environment

} //@namespace dwl


#endif
