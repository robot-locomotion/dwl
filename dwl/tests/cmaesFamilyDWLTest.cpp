#include <dwl/model/OptimalControlModel.h>
#include <dwl/solver/OptimizationSolver.h>
#include <dwl/solver/cmaesSOFamily.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
	dwl::solver::OptimizationSolver* solver = new dwl::solver::cmaesSOFamily();
	dwl::model::OptimalControlModel optimal_control;
	solver->setOptimizationModel(&optimal_control);

	dwl::model::DynamicalSystem* dynamical_system = new dwl::model::HS071DynamicalSystem();
	dynamical_system->defineAsSoftConstraint();
	dwl::model::Cost* cost = new dwl::model::HS071Cost();

	optimal_control.addDynamicalSystem(dynamical_system);
	optimal_control.addCost(cost);

	solver->init();
	solver->compute();

	return 0;
}
