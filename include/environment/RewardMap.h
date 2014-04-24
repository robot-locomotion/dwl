#ifndef DWL_RewardMap_H
#define DWL_RewardMap_H

#include <environment/Feature.h>
#include <vector>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{

class RewardMap
{
	public:
		RewardMap();
		virtual ~RewardMap();

		void addFeature(Feature* feature);

		void removeFeature(std::string feature_name);//{features_}

		virtual void compute() {};

	protected:
		std::vector<Feature*> features_;





};

} //@namespace environment

} //@namespace dwl


#endif
