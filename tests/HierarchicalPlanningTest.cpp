#include <planning/DijkstrapAlgorithm.h>
#include <planning/HierarchicalPlanning.h>
#include <planning/WholeBodyLocomotion.cpp>
#include <planning/CostMap.cpp>
#include <hyq/KinematicConstraints.cpp>
#include <hyq/StabilityConstraints.cpp>
#include <hyq/StateCost.cpp>
#include <Eigen/Dense>

#include <environment/RewardMap.h>
#include <environment/SlopeFeature.h>


int main(int argc, char **argv)
{
	//  Setup of the locomotion approach
	dwl::WholeBodyLocomotion locomotor;

	// Initialization of planning algorithm, which includes the initizalization and setup of solver algorithm
	dwl::planning::Solver* solver_ptr = new dwl::planning::DijkstrapAlgorithm();
	dwl::planning::PlanningOfMotionSequences* planning_ptr = new dwl::planning::HierarchicalPlanning();
	planning_ptr->reset(solver_ptr);

	dwl::planning::Constraint* kin_constraint_ptr = new dwl::hyq::KinematicConstraints();
	dwl::planning::Constraint* stab_constraint_ptr = new dwl::hyq::StabilityConstraints();
	dwl::planning::Cost* state_cost_ptr = new dwl::hyq::StateCost();
	dwl::planning::Cost* cost_map_ptr = new dwl::planning::CostMap();

	// Setting up the planner algorithm in the locomotion approach
	locomotor.reset(planning_ptr);
	locomotor.addConstraint(kin_constraint_ptr);
	locomotor.addConstraint(stab_constraint_ptr);
	locomotor.addCost(state_cost_ptr);
	locomotor.addCost(cost_map_ptr);
	locomotor.removeConstraint("fake");
	locomotor.removeConstraint(kin_constraint_ptr->getName());
	//locomotor.removeCost(cost_ptr->getName());

	// Initizalization and computing of the whole-body locomotion problem
	std::vector<double> start, goal;
	locomotor.init(start, goal);
	cost_map_ptr->setCostMap();
	locomotor.computePlan();
	locomotor.changeGoal(goal);

/*
	printf("---------- RewardMap ------------\n");

	// Setup of reward map
	dwl::environment::RewardMap reward_map;

	// Initialization of reward map algorithm
	dwl::environment::Feature* slope_ptr = new dwl::environment::SlopeFeature();
	reward_map.addFeature(slope_ptr);
	reward_map.removeFeature("fake");
*/

    return 0;
}
