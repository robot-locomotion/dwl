#include <planning/Dijkstrap.h>
#include <planning/HierarchicalPlanning.h>
#include <planning/WholeBodyLocomotion.cpp>

#include <robot/KinematicConstraints.cpp>
#include <robot/StabilityConstraints.cpp>
#include <robot/StateCost.cpp>
#include <Eigen/Dense>

#include <environment/EnvironmentInformation.h>
#include <environment/RewardMap.h>
#include <environment/SlopeFeature.h>

template <typename Weight, typename Vertex>
struct pair_first_less
{
    bool operator()(std::pair<Weight,Vertex> vertex_1, std::pair<Weight,Vertex> vertex_2)
    {
        return vertex_1.first < vertex_2.first;
    }
};

int main(int argc, char **argv)
{
	//  Setup of the locomotion approach
	dwl::WholeBodyLocomotion locomotor;

	// Initialization of planning algorithm, which includes the initialization and setup of solver algorithm
	dwl::planning::Solver* solver_ptr = new dwl::planning::Dijkstrap();
	dwl::planning::PlanningOfMotionSequences* planning_ptr = new dwl::planning::HierarchicalPlanning();
	dwl::environment::EnvironmentInformation* environment_ptr;
	planning_ptr->reset(solver_ptr, environment_ptr);

	dwl::planning::Constraint* kin_constraint_ptr = new dwl::robot::KinematicConstraints();
	dwl::planning::Constraint* stab_constraint_ptr = new dwl::robot::StabilityConstraints();
	dwl::planning::Cost* state_cost_ptr = new dwl::robot::StateCost();

	// Setting up the planner algorithm in the locomotion approach
	locomotor.reset(planning_ptr);
	locomotor.addConstraint(kin_constraint_ptr);
	locomotor.addConstraint(stab_constraint_ptr);
	locomotor.addCost(state_cost_ptr);
	locomotor.removeConstraint("fake");
	locomotor.removeConstraint(kin_constraint_ptr->getName());
	//locomotor.removeCost(cost_ptr->getName());

	// Initialization and computing of the whole-body locomotion problem
	dwl::Pose start, goal, current;
	std::vector<dwl::RewardCell> reward_map;
	locomotor.init();

	Eigen::Vector3d robot_state = Eigen::Vector3d::Zero();
	locomotor.compute(current);
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
	std::set< std::pair<double, int>, pair_first_less<double, int> > vertex_queue;
	vertex_queue.insert(std::pair<double, int>(5, 1));
	vertex_queue.insert(std::pair<double, int>(2.5, 2));
	vertex_queue.insert(std::pair<double, int>(0.5, 3));
	vertex_queue.insert(std::pair<double, int>(0.25, 4));
	vertex_queue.insert(std::pair<double, int>(5.5, 5));
//	while (!vertex_queue.empty()) {
	for (int i = 0; i < 2; i++) {
		int u = vertex_queue.begin()->second;
		double weight = vertex_queue.begin()->first;
		vertex_queue.erase(vertex_queue.begin());

		std::cout << "Weight = " << weight << " ";
	}



	std::map<char,int> mymap;
	  std::map<char,int>::iterator it;

	  mymap['a']=50;
	  mymap['b']=100;
	  mymap['c']=150;
	  mymap['d']=200;

	  it=mymap.find('b');
	  mymap.erase (it);
	  mymap.erase (mymap.find('d'));

	  // print content:
	  std::cout << "elements in mymap:" << '\n';
	  std::cout << "a => " << mymap.find('a')->second << '\n';
	  std::cout << "c => " << mymap.find('c')->second << '\n';
	  std::cout << "z => " << mymap.find('z')->second << '\n';

	  if (mymap.find('a')->first == 'a')
		  std::cout << "Detect!" << std::endl;

    return 0;
}
