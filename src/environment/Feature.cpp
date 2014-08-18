#include <environment/Feature.h>


namespace dwl
{

namespace environment
{

Feature::Feature() : weight_(1), gain_(1)
{

}


Feature::~Feature()
{

}


std::string Feature::getName()
{
	return name_;
}


void Feature::setWeight(double weight)
{
	weight_ = weight;
}


void Feature::getWeight(double& weight)
{
	weight = weight_;
}


void Feature::setNeighboringArea(double min_x, double max_x, double min_y, double max_y, double resolution)
{
	neightboring_area_.max_x = max_x;
	neightboring_area_.min_x = min_x;
	neightboring_area_.max_y = max_y;
	neightboring_area_.min_y = min_y;
	neightboring_area_.grid_resolution = resolution;
}


void Feature::setGainFeature(double gain)
{
	gain_ = gain;
}

} //@namespace environment
} //@namespace dwl
