#include <planning/DijkstrapAlgorithm.h>


namespace dwl
{

namespace planning
{

DijkstrapAlgorithm::DijkstrapAlgorithm()
{
	name_ = "dijkstrap";
}


DijkstrapAlgorithm::~DijkstrapAlgorithm()
{

}


bool DijkstrapAlgorithm::init()
{
	printf("Initialized the Dijkstrap algortihm\n");
	return true;
}


bool DijkstrapAlgorithm::compute(Eigen::MatrixXd& solution)
{
	printf("Computing the Dijkstrap algorithm\n");

	Eigen::VectorXd constraint_value, state_value;
	active_constraints_[0]->get(constraint_value, state_value);

	double cost;
	costs_[0]->get(cost, state_value);

	return true;
}


} //@namespace planning

} //@namespace dwl
