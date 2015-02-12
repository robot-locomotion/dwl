#ifndef DWL_LearningDiscoveryOfNewBehaviors_H
#define DWL_LearningDiscoveryOfNewBehaviors_H

#include <vector>


namespace dwl
{

namespace learning
{

class LearningDiscoveryOfNewBehaviors
{

	public:
		/** @brief Constructor function */
		LearningDiscoveryOfNewBehaviors() {}

		/** @brief Destructor function */
		virtual ~LearningDiscoveryOfNewBehaviors() {}

		virtual bool init();
		virtual bool compute();


	private:
		std::vector<double> reward_;
};

} //@namespace learning
} //@namespace dwl


#endif
