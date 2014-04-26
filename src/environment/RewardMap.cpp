#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap()
{

}


RewardMap::~RewardMap()
{

}


void RewardMap::addFeature(Feature* feature)
{
	printf(GREEN "Adding the %s feature\n" COLOR_RESET, feature->getName().c_str());
	pthread_mutex_lock(&environment_lock_);
	features_.push_back(feature);
	pthread_mutex_unlock(&environment_lock_);
}


void RewardMap::removeFeature(std::string feature_name)
{
	for (int i = 0; i < features_.size(); i++) {
		if (feature_name == features_[i]->getName().c_str()) {
			printf(GREEN "Removing the %s feature\n" COLOR_RESET, features_[i]->getName().c_str());
			pthread_mutex_lock(&environment_lock_);
			features_.erase(features_.begin() + i);
			pthread_mutex_unlock(&environment_lock_);

			return;
		}
		else if (i == features_.size() - 1) {
			printf(YELLOW "Could not remove the %s feature\n" COLOR_RESET, feature_name.c_str());
		}
	}
}


} //@namepace dwl

} //@namespace environment
