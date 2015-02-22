#include <solver/Dijkstrap.h>
#include <locomotion/HierarchicalPlanning.h>
#include <locomotion/WholeBodyLocomotion.cpp>

#include <robot/Robot.cpp>
#include <constraint/KinematicConstraints.cpp>
#include <constraint/StabilityConstraints.cpp>
#include <robot/StateCost.cpp>
#include <Eigen/Dense>

#include <environment/EnvironmentInformation.h>
#include <environment/RewardMap.h>
#include <environment/SlopeFeature.h>

#include <model/WholeBodyKinematics.h>
#include <model/WholeBodyDynamics.h>
#include <robot/HyQWholeBodyKinematics.h>
#include <robot/HyLWholeBodyDynamics.h>
#include <iit/rbd/rbd.h>



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
//	//  Setup of the locomotion approach
//	dwl::WholeBodyLocomotion locomotor;
//
//	// Initialization of planning algorithm, which includes the initialization and setup of solver algorithm
//	dwl::robot::Robot* robot_ptr = NULL;
//	dwl::solver::Solver* solver_ptr = new dwl::solver::Dijkstrap();
//	dwl::locomotion::PlanningOfMotionSequence* planning_ptr = new dwl::locomotion::HierarchicalPlanning();
//	dwl::environment::EnvironmentInformation* environment_ptr = NULL;
//	planning_ptr->reset(robot_ptr, solver_ptr, environment_ptr);
//
//	dwl::constraint::Constraint* kin_constraint_ptr = new dwl::constraint::KinematicConstraints();
//	dwl::constraint::Constraint* stab_constraint_ptr = new dwl::constraint::StabilityConstraints();
//	dwl::locomotion::Cost* state_cost_ptr = new dwl::robot::StateCost();
//
//	// Setting up the planner algorithm in the locomotion approach
//	locomotor.reset(planning_ptr);
//	locomotor.addConstraint(kin_constraint_ptr);
//	locomotor.addConstraint(stab_constraint_ptr);
//	locomotor.addCost(state_cost_ptr);
//	locomotor.removeConstraint("fake");
//	locomotor.removeConstraint(kin_constraint_ptr->getName());
//	//locomotor.removeCost(cost_ptr->getName());
//
//	// Initialization and computing of the whole-body locomotion problem
//	dwl::Pose start, goal, current;
//	std::vector<dwl::RewardCell> reward_map;
//	locomotor.init();
//
//	locomotor.compute(current);
//
//	std::set< std::pair<double, int>, pair_first_less<double, int> > vertex_queue;
//	vertex_queue.insert(std::pair<double, int>(5, 1));
//	vertex_queue.insert(std::pair<double, int>(2.5, 2));
//	vertex_queue.insert(std::pair<double, int>(0.5, 3));
//	vertex_queue.insert(std::pair<double, int>(0.25, 4));
//	vertex_queue.insert(std::pair<double, int>(5.5, 5));
//	for (int i = 0; i < 2; i++) {
//		double weight = vertex_queue.begin()->first;
//		vertex_queue.erase(vertex_queue.begin());
//
//		std::cout << "Weight = " << weight << " ";
//	}
//
//	std::map<char,int> mymap;
//	std::map<char,int>::iterator it;
//
//	mymap['a']=50;
//	mymap['b']=100;
//	mymap['c']=150;
//	mymap['d']=200;
//
//	it=mymap.find('b');
//	mymap.erase (it);
//	mymap.erase (mymap.find('d'));
//
//	// print content:
//	std::cout << "elements in mymap:" << '\n';
//	std::cout << "a => " << mymap.find('a')->second << '\n';
//	std::cout << "c => " << mymap.find('c')->second << '\n';
//	std::cout << "z => " << mymap.find('z')->second << '\n';
//
//	if (mymap.find('a')->first == 'a')
//		std::cout << "Detect!" << std::endl;

/*
	dwl::model::WholeBodyKinematics* kin_ptr = new dwl::robot::HyQWholeBodyKinematics();

	Eigen::VectorXd base_pos = Eigen::VectorXd::Zero(6);
	iit::HyQ::JointState jnt_pos;
	jnt_pos << -0.2174, 0.5427, -1.2621, -0.29997, 0.6941, -1.2098, -0.2901, -0.6674, 1.1602, -0.2689, -0.5008, 1.2326;
	Eigen::VectorXd pos = Eigen::VectorXd::Zero(18);
	pos << base_pos, jnt_pos;

	kin_ptr->init();
	kin_ptr->updateState(pos, pos);

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;

	dwl::EndEffectorSelector effector_set;
	effector_set["LF_foot"] = true;
	effector_set["RF_foot"] = true;
//	kin_ptr->computeFloatingBaseJacobian(jacobian, dwl::model::Full);
//	kin_ptr->computeFixedBaseJacobian(jacobian, active_contact, dwl::model::Full);
	kin_ptr->computeWholeBodyJacobian(jacobian, effector_set, dwl::model::Full);
*/


	dwl::model::WholeBodyDynamics* dyn_ptr = new dwl::robot::HyLWholeBodyDynamics();

	Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> base_vel = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> base_accel = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> wrench;

	Eigen::VectorXd tau, q, qd, qdd;
	q = Eigen::VectorXd::Zero(4);
	q << 0, 0.75, -1.5;
	qd = Eigen::VectorXd::Zero(4);
	qdd = Eigen::VectorXd::Zero(4);

	dyn_ptr->computeWholeBodyInverseDynamics(wrench, tau, g, base_vel, base_accel, q, qd, qdd);

    return 0;
}
