#include <solver/OptimizationSolver.h>
#include <solver/IpoptNLP.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
//	dwl::solver::Solver* solver = new dwl::solver::IpoptNLP();
	dwl::solver::IpoptNLP* ipopt_solver = new dwl::solver::IpoptNLP();
	dwl::solver::OptimizationSolver* solver = ipopt_solver;

	dwl::model::DynamicalSystem* dynamical_system = new dwl::model::HS071DynamicalSystem();
	dwl::model::Cost* cost = new dwl::model::HS071Cost();


	dwl::rbd::FloatingBaseSystem system;
	system.setJointDoF(4);
	system.setTypeOfDynamicSystem(dwl::rbd::FixedBase);
	dynamical_system->setFloatingBaseSystem(&system);

//	ipopt_solver->getIpopt().getOptimizationModel().addDynamicalSystem(dynamical_system);
//	ipopt_solver->getIpopt().getOptimizationModel().addCost(cost);
	solver->getOptimizationModel().addDynamicalSystem(dynamical_system);
	solver->getOptimizationModel().addCost(cost);

//	dwl::model::OptimizationModel model;
//	model.addDynamicalSystem(dynamical_system);
//	model.addCost(cost);
//	solver->setModel(&model);


//	planning_ptr_->reset(&robot_, solver_, environment_);

	solver->init();
	solver->compute();

	return 0;
}
