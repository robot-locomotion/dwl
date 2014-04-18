#include <planning/HierarchicalPlanning.h>



namespace dwl
{

namespace planning
{


HierarchicalPlanning::HierarchicalPlanning()
{
	name_ = "hierarchical";
}


bool HierarchicalPlanning::init(std::vector<double> start, std::vector<double> goal)
{
	printf("Initialized the HierarchicalPlanning algortihm\n");
	solver_->init();

	return true;
}


bool HierarchicalPlanning::compute()
{
	printf("Computing the HierarchicalPlanning algorithm\n");

	Eigen::MatrixXd solution;
	solver_->compute(solution);

	return true;
}


} //@namespace planning

} //@namespace dwl
