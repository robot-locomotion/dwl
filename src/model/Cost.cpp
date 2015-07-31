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


void Cost::setLocomotionStateWeights(LocomotionState& weights)
{
	locomotion_weights_ = weights;
}


std::string Cost::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
