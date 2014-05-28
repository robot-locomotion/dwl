#include <planning/HierarchicalPlanning.h>


namespace dwl
{

namespace planning
{


HierarchicalPlanning::HierarchicalPlanning() : gridmap_(0.04, 0.02), source_id_(0), target_id_(0)
{
	name_ = "hierarchical";
}


HierarchicalPlanning::~HierarchicalPlanning()
{

}


bool HierarchicalPlanning::init()
{
	printf("Initialized the HierarchicalPlanning algortihm\n");

	solver_->init();

	return true;
}


void HierarchicalPlanning::update(BodyPose start, BodyPose goal)
{
	// Converting the start and goal position to vertex ids
	dwl::environment::Key start_key, goal_key;
	gridmap_.coordToKeyChecked(start_key, (Eigen::Vector2d) start.position.head(2));
	gridmap_.coordToKeyChecked(goal_key, (Eigen::Vector2d) goal.position.head(2));
	source_id_ = gridmap_.gridmapKeyToVertex(start_key);
	target_id_ = gridmap_.gridmapKeyToVertex(goal_key);
}


bool HierarchicalPlanning::compute()
{
	printf("Computing the HierarchicalPlanning algorithm\n");

	SolverInterface solver;


	solver.searcher.source = source_id_;
	solver.searcher.target = target_id_;

	solver_->compute(solver);

	std::list<Vertex> path = solver_->getShortestPath(target_id_);
	std::cout << "Total cost to " << target_id_ << " from " << source_id_ << ": " << solver_->getMinimumCost() << std::endl;

	//std::cout << "Total cost to " << vertex_names[6] << ": " << solver_->getMinimumCost() << std::endl;
	std::list<Vertex>::iterator path_iter = path.begin();
	std::cout << "Path: ";
	for( ; path_iter != path.end(); path_iter++) {
		std::cout << *path_iter << " "; //vertex_names[*path_iter] << " ";
	}
	std::cout << std::endl;


	return true;
}


} //@namespace planning

} //@namespace dwl
