#include <environment/SlopeFeature.h>


namespace dwl
{

namespace environment
{


SlopeFeature::SlopeFeature()
{
	name_ = "slope";
}

SlopeFeature::~SlopeFeature()
{

}

void SlopeFeature::compute()
{
	printf("Computing the slope feature\n");
}


} //@namespace environment

} //@namespace dwl
