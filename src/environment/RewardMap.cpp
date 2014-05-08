#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap() : is_added_feature_(false), is_added_search_area_(false)
{

}


RewardMap::~RewardMap()
{
	if (is_added_feature_) {
	for (std::vector<Feature*>::iterator i = features_.begin(); i != features_.end(); i++)
		delete *i;
	}
}


void RewardMap::addFeature(Feature* feature)
{
	printf(GREEN "Adding the %s feature\n" COLOR_RESET, feature->getName().c_str());
	//pthread_mutex_lock(&environment_lock_);
	features_.push_back(feature);
	//pthread_mutex_unlock(&environment_lock_); //TODO: I had problems when re-run the code. For this reason I had to remove these lines. Figure out why!!!
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


void RewardMap::addSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_size)
{
	SearchArea search_area;
	search_area.min_x = min_x;
	search_area.max_x = max_x;
	search_area.min_y = min_y;
	search_area.max_y = max_y;
	search_area.min_z = min_z;
	search_area.max_z = max_z;
	search_area.grid_size = grid_size;

	search_areas_.push_back(search_area);
	is_added_search_area_ = true;
}


void RewardMap::setNeighboringArea(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z)
{
	neighboring_area_.min_x = min_x;
	neighboring_area_.max_x = max_x;
	neighboring_area_.min_y = min_y;
	neighboring_area_.max_y = max_y;
	neighboring_area_.min_z = min_z;
	neighboring_area_.max_z = max_z;
}


} //@namepace dwl

} //@namespace environment
