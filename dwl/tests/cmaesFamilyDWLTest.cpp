#include <dwl/ocp/OptimalControl.h>
#include <dwl/ocp/PreviewOptimization.h>
#include <dwl/solver/OptimizationSolver.h>
#include <dwl/solver/cmaesSOFamily.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
	bool op = true;
	dwl::solver::OptimizationSolver* solver = new dwl::solver::cmaesSOFamily<>();
	dwl::ocp::OptimalControl optimal_control;
	dwl::ocp::PreviewOptimization preview_opt;
	if (op) {
		solver->setOptimizationModel(&optimal_control);

		dwl::ocp::DynamicalSystem* dynamical_system = new dwl::model::HS071DynamicalSystem();
		dynamical_system->defineAsSoftConstraint();
		dwl::ocp::Cost* cost = new dwl::model::HS071Cost();

		optimal_control.addDynamicalSystem(dynamical_system);
		optimal_control.addCost(cost);
	} else {
		solver->setOptimizationModel(&preview_opt);
	}

	solver->init();
	solver->setFromConfigFile("../config/cmaes_config.yaml");
	solver->compute();

	return 0;
}
