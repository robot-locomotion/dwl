#include <model/Cost.h>


namespace dwl
{

namespace model
{

Cost::Cost()
{

}


Cost::~Cost()
{

}


void Cost::setWeights(LocomotionState& weights)
{
	locomotion_weights_ = weights;
}


std::string Cost::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
