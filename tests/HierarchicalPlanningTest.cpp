#include <planning/DijkstrapAlgorithm.h>
#include <planning/HierarchicalPlanning.h>
#include <planning/WholeBodyLocomotion.cpp>
#include <hyq/KinematicConstraints.cpp>
#include <hyq/StateCost.cpp>
#include <Eigen/Dense>


int main(int argc, char **argv)
{
	//  Setup of the locomotion approach
	dwl::WholeBodyLocomotion locomotor;

	// Initialization of planning algorithm, which includes the initizalization and setup of solver algorithm
	dwl::planning::Solver* solver_ptr = new dwl::planning::DijkstrapAlgorithm();
	dwl::planning::PlanningOfMotionSequences* planning_ptr = new dwl::planning::HierarchicalPlanning();
	planning_ptr->reset(solver_ptr);

	dwl::planning::Constraint* constraint_ptr = new dwl::hyq::KinematicConstraints();
	dwl::planning::Cost* cost_ptr = new dwl::hyq::StateCost();

	// Setting up the planner algorithm in the locomotion approach
	locomotor.reset(planning_ptr);
	locomotor.addConstraint(constraint_ptr);
	locomotor.addCost(cost_ptr);

	// Initizalization and computing of the whole-body locomotion problem
	locomotor.init();
	locomotor.computePlan();

    return 0;
}
