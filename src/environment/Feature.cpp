#include <environment/Feature.h>


namespace dwl
{

namespace environment
{


Feature::Feature()
{

}


Feature::~Feature()
{

}


void Feature::addCellToRewardMap(double reward, Terrain terrain_info)
{
	Cell cell;
	cell.position = terrain_info.position;
	cell.reward = reward;
	reward_gridmap_.push_back(cell);
}

std::string Feature::getName()
{
	return name_;
}


} //@namespace environment

} //@namespace dwl
