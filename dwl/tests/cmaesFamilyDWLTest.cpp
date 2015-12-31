#include <dwl/solver/OptimizationSolver.h>
#include <dwl/solver/CMAES.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
	dwl::solver::cmaesSOFamily* cmaes_solver = new dwl::solver::cmaesSOFamily();
	dwl::solver::OptimizationSolver* solver = cmaes_solver;

//	dwl::model::DynamicalSystem* dynamical_system = new dwl::model::HS071DynamicalSystem();
	dwl::model::Cost* cost = new dwl::model::HS071Cost();

//	solver->getOptimizationModel().addDynamicalSystem(dynamical_system);
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
