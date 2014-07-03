#include <environment/Feature.h>


namespace dwl
{

namespace environment
{


Feature::Feature() : weight_(1)
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

} //@namespace environment

} //@namespace dwl
