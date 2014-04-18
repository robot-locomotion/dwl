#ifndef DWL_HierarchicalPlanning_H
#define DWL_HierarchicalPlanning_H


#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{

namespace planning
{


/* Derived class: HierarchicalPlanning */
class HierarchicalPlanning : public dwl::planning::PlanningOfMotionSequences
{
	public:
		/** @brief Constructor function */
		HierarchicalPlanning();

		/** @brief Destructor function */
		~HierarchicalPlanning() {}

		bool init(std::vector<double> start, std::vector<double> goal);
		bool compute();

	private:

		std::vector<Eigen::Vector3d> body_path_;


}; //@class HierarchicalPlanning

} //@namespace dwl
}

#endif
