#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap() : is_added_feature_(false)
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
	is_added_feature_ = true;
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


void RewardMap::setSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_size)
{
	search_area_.min_x = min_x;
	search_area_.max_x = max_x;
	search_area_.min_y = min_y;
	search_area_.max_y = max_y;
	search_area_.min_z = min_z;
	search_area_.max_z = max_z;
	search_area_.grid_size = grid_size;
}



} //@namepace dwl

} //@namespace environment
