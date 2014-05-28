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
	SolverInterface solver;

	solver.searcher.source = source_id_;
	solver.searcher.target = target_id_;
	solver_->compute(solver);

	std::list<Vertex> path = solver_->getShortestPath(target_id_);


	//TODO only for debugging
	dwl::environment::Key key;
	gridmap_.vertexToGridmapKey(key, source_id_);
	double x = gridmap_.keyToCoord((unsigned short int) key.key[0], true);
	double y = gridmap_.keyToCoord((unsigned short int) key.key[1], true);
	gridmap_.vertexToGridmapKey(key, target_id_);
	double x_t = gridmap_.keyToCoord((unsigned short int) key.key[0], true);
	double y_t = gridmap_.keyToCoord((unsigned short int) key.key[1], true);
	std::cout << "Total cost to [" << x_t << " " << y_t << "] from [" << x << " " << y << "]: " << solver_->getMinimumCost() << std::endl;

	//TODO Some line for debugging
	std::list<Vertex>::iterator path_iter = path.begin();
	std::cout << "Path: ";
	for( ; path_iter != path.end(); path_iter++) {
		Eigen::Vector3d body_path = Eigen::Vector3d::Zero();

		dwl::environment::Key path_key;
		gridmap_.vertexToGridmapKey(path_key, *path_iter);
		double x = gridmap_.keyToCoord(path_key.key[0], true);
		double y = gridmap_.keyToCoord(path_key.key[1], true);

		body_path[0] = x;
		body_path[1] = y;
		body_path[2] = 0;

		body_path_.push_back(body_path);

		std::cout << "[" << x << " " << y << "] | ";
	}
	std::cout << std::endl;


	return true;
}


} //@namespace planning

} //@namespace dwl
