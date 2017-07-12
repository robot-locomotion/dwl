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
		dwl::ocp::SoftConstraintProperties properties(10000, 0.1, dwl::ocp::QUADRATIC);
		dynamical_system->setSoftProperties(properties);
		dwl::ocp::Cost* cost = new dwl::model::HS071Cost();

		optimal_control.addDynamicalSystem(dynamical_system);
		optimal_control.addCost(cost);
	} else {
		solver->setOptimizationModel(&preview_opt);
	}

	solver->setFromConfigFile("../config/cmaes_config.yaml");
	solver->init();
	solver->compute();
	Eigen::VectorXd sol = solver->getSolution();
	printf(BLUE "The solution is: " COLOR_RESET);
	std::cout << sol.transpose() << std::endl;


	return 0;
}
