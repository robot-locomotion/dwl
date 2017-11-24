#include <dwl/ocp/OptimalControl.h>
#include <dwl/solver/OptimizationSolver.h>
#include <dwl/solver/IpoptNLP.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
	dwl::solver::OptimizationSolver* solver = new dwl::solver::IpoptNLP();
	dwl::ocp::OptimalControl optimal_control;
	solver->setOptimizationModel(&optimal_control);

	dwl::ocp::DynamicalSystem* dynamical_system = new dwl::model::HS071DynamicalSystem();
	dwl::ocp::Cost* cost = new dwl::model::HS071Cost();

	optimal_control.addDynamicalSystem(dynamical_system);
	optimal_control.addCost(cost);

	solver->setFromConfigFile("../config/ipopt_config.yaml");

	std::clock_t startcputime = std::clock();
	solver->init();
	solver->compute();
	Eigen::VectorXd sol = solver->getSolution();
	double cpu_duration =
				(std::clock() - startcputime) / (double) CLOCKS_PER_SEC;
	std::cout << "  Computation time: " << cpu_duration << " (microsecs, CPU time)" << std::endl;
	printf(BLUE "The solution is: " COLOR_RESET);
	std::cout << sol.transpose() << std::endl;

	return 0;
}
