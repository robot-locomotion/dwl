#include <dwl/solver/IpoptNLP.h>


namespace dwl
{

namespace solver
{

IpoptNLP::IpoptNLP() : initialized_(false), print_level_(5),
		print_freq_iter_(1), outfile_(false), filename_("ipopt.opt"),
		file_print_level_(5), convergence_tol_(1e-7), max_iter_(-1),
		dual_inf_tol_(1.), constr_viol_tol_(0.0001), compl_viol_tol_(0.0001),
		acceptable_tol_(1e-6), acceptable_iter_(15), mu_strategy_("adaptive")
{
	name_ = "IpoptNLP";
}


IpoptNLP::~IpoptNLP()
{

}


void IpoptNLP::setFromConfigFile(std::string filename)
{
	// Checking if the solver was initialized before setting it from config file
	bool reinit = false;
	if (initialized_) {
		initialized_ = false;
		reinit = true;
	}

	// Yaml reader
	YamlWrapper yaml_reader(filename);

	// Parsing the configuration file
	std::string ipopt_ns = "ipopt";
	printf("Reading the configuration parameters from the %s namespace\n",
			ipopt_ns.c_str());

	// Getting the different nodes
	YamlNamespace output_ns = {ipopt_ns, "output"};
	YamlNamespace output_file_ns = {ipopt_ns, "output", "output_file"};
	YamlNamespace termination_ns = {ipopt_ns, "termination"};
	YamlNamespace barrier_ns = {ipopt_ns, "barrier"};

	// Output parameters
	// Reading and setting up the print level
	int print_level;
	if (yaml_reader.read(print_level, "print_level", output_ns))
		setPrintLevel(print_level);

	// Reading and setting up the print frequency iteration
	int print_frequency_iter;
	if (yaml_reader.read(print_frequency_iter, "print_frequency_iter", output_ns))
		setPrintFrequencyIteration(print_frequency_iter);

	// Reading and setting up the out file parameters
	std::string option_file_name;
	if (yaml_reader.read(option_file_name, "option_file_name", output_file_ns)) {
		setFilename(option_file_name);

		int file_print_level;
		if (yaml_reader.read(file_print_level, "file_print_level", output_file_ns))
			setFilePrintLevel(file_print_level);
	}

	// Termination parameters
	// Reading and setting up the convergence tolerance
	double tol;
	if (yaml_reader.read(tol, "tol", termination_ns))
		setConvergenceTolerance(tol);

	// Reading and setting up the allowed number of iteration
	int max_iter;
	if (yaml_reader.read(max_iter, "max_iter", termination_ns))
		setMaxIteration(max_iter);

	// Reading and setting up the desired threshold for the dual infeasibility
	double dual_inf_tol;
	if (yaml_reader.read(dual_inf_tol, "dual_inf_tol", termination_ns))
		setDualInfeasibilityTolerance(dual_inf_tol);

	// Reading and setting up the desired threshold for the constraint violation
	double constr_viol_tol;
	if (yaml_reader.read(constr_viol_tol, "constr_viol_tol", termination_ns))
		setConstraintViolationTolerance(constr_viol_tol);

	// Reading and setting up the desired threshold for the complementarity conditions
	double compl_inf_tol;
	if (yaml_reader.read(compl_inf_tol, "compl_inf_tol", termination_ns))
		setComplementaryTolerance(compl_inf_tol);

	// Reading and setting up the "acceptable" convergence tolerance (relative)
	double acceptable_tol;
	if (yaml_reader.read(acceptable_tol, "acceptable_tol", termination_ns))
		setAcceptableConvergenceTolerance(acceptable_tol);

	// Reading and setting up the number of "acceptable" iterates before triggering termination
	int acceptable_iter;
	if (yaml_reader.read(acceptable_iter, "acceptable_iter", termination_ns))
		setAcceptableIterations(acceptable_iter);

	// Barrier parameters
	// Reading and setting up the out file parameters
	std::string mu_strategy;
	if (yaml_reader.read(mu_strategy, "mu_strategy", barrier_ns))
		setMuStrategy(mu_strategy);

	// Re-initialization of the solver if it was initialized
	if (reinit)
		init();
}


void IpoptNLP::setPrintLevel(int print_level)
{
	print_level_ = print_level;

	if (initialized_)
		app_->Options()->SetIntegerValue("print_level", print_level_);
}


void IpoptNLP::setPrintFrequencyIteration(int print_freq_iter)
{
	print_freq_iter_ = print_freq_iter;

	if (initialized_)
		app_->Options()->SetIntegerValue("print_frequency_iter", print_freq_iter_);
}


void IpoptNLP::setFilename(std::string filename)
{
	outfile_ = true;
	filename_ = filename;

	if (initialized_)
		app_->Options()->SetStringValue("option_file_name", filename_);
}


void IpoptNLP::setFilePrintLevel(int file_print_level)
{
	file_print_level_ = file_print_level;

	if (initialized_)
		app_->Options()->SetIntegerValue("file_print_level", file_print_level_);
}


void IpoptNLP::setConvergenceTolerance(double tolerance)
{
	convergence_tol_ = tolerance;

	if (initialized_)
		app_->Options()->SetNumericValue("tol", tolerance);
}


void IpoptNLP::setMaxIteration(int max_iter)
{
	max_iter_ = max_iter;

	if (initialized_) {
		if (max_iter_ == -1)
			app_->Options()->SetIntegerValue("max_iter", std::numeric_limits<int>::max());
		else
			app_->Options()->SetIntegerValue("max_iter", max_iter_);
	}
}


void IpoptNLP::setDualInfeasibilityTolerance(double tolerance)
{
	dual_inf_tol_ = tolerance;

	if (initialized_)
		app_->Options()->SetNumericValue("dual_inf_tol", dual_inf_tol_);
}


void IpoptNLP::setConstraintViolationTolerance(double tolerance)
{
	constr_viol_tol_ = tolerance;

	if (initialized_)
		app_->Options()->SetNumericValue("constr_viol_tol", constr_viol_tol_);
}


void IpoptNLP::setComplementaryTolerance(double tolerance)
{
	compl_viol_tol_ = tolerance;

	if (initialized_)
		app_->Options()->SetNumericValue("compl_inf_tol", compl_viol_tol_);
}


void IpoptNLP::setAcceptableConvergenceTolerance(double tolerance)
{
	acceptable_tol_ = tolerance;

	if (initialized_)
		app_->Options()->SetNumericValue("acceptable_tol", acceptable_tol_);
}


void IpoptNLP::setAcceptableIterations(int iterations)
{
	acceptable_iter_ = iterations;

	if (initialized_)
		app_->Options()->SetIntegerValue("acceptable_iter", acceptable_iter_);
}


void IpoptNLP::setMuStrategy(std::string mu_strategy)
{
	mu_strategy_ = mu_strategy;

	if (initialized_)
		app_->Options()->SetStringValue("mu_strategy", mu_strategy_);
}


bool IpoptNLP::init()
{
	// Setting the optimization model to Ipopt wrapper
	ipopt_.setOptimizationModel(model_);

	// Create a new instance of your NLP
	nlp_ptr_ = &ipopt_;

	// Creating a new instance of IpoptApplication (use a SmartPtr, not raw). We are using the
	// factory, since this allows us to compile this example with an Ipopt Windows DLL
	app_ = IpoptApplicationFactory();
	app_->RethrowNonIpoptException(true);

	// Setting the previous parameters
	initialized_ = true;
	setPrintLevel(print_level_);
	setPrintFrequencyIteration(print_freq_iter_);
	if (outfile_) {
		setFilename(filename_);
		setFilePrintLevel(file_print_level_);
	}
	setConvergenceTolerance(convergence_tol_);
	setMaxIteration(max_iter_);
	setDualInfeasibilityTolerance(dual_inf_tol_);
	setConstraintViolationTolerance(constr_viol_tol_);
	setComplementaryTolerance(compl_viol_tol_);
	setAcceptableConvergenceTolerance(acceptable_tol_);
	setAcceptableIterations(acceptable_iter_);
	setMuStrategy(mu_strategy_);

	// Change some options (do not touch these)
	app_->Options()->SetNumericValue("tol", 1e-7);
//	app_->Options()->SetNumericValue("acceptable_tol", 1e-2);
	app_->Options()->SetStringValue("mu_strategy", "adaptive");
//	app_->Options()->SetStringValue("output_file", "ipopt.out");
	app_->Options()->SetIntegerValue("max_iter", std::numeric_limits<int>::max());
	app_->Options()->SetIntegerValue("print_level", 5);

	// Computing Hessian numerically (do not need to implement)
	app_->Options()->SetStringValue("hessian_approximation", "limited-memory");

	// Computing Hessian numerically (do not need to implement)
	app_->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");

	app_->Options()->SetStringValue("warm_start_init_point", "yes");

//	app_->Options()->SetNumericValue("dual_inf_tol", 1000);
//	app_->Options()->SetNumericValue("constr_viol_tol", 0.1);
//	app_->Options()->SetNumericValue("compl_inf_tol", 0.1);

	// Initialize the IpoptApplication and process the options
	Ipopt::ApplicationReturnStatus status;
	status = app_->Initialize();
	if (status != Ipopt::Solve_Succeeded) {
		printf("\n\n*** Error during initialization!\n");
		return (int) status;
	}

	return true;
}


bool IpoptNLP::compute(double allocated_time_secs)
{
	// Setting the initial time
	clock_t started_time = clock();

	// Ask Ipopt to solve the problem
	Ipopt::ApplicationReturnStatus status;

	// Computing the optimization problem
	bool solved = false;
	double current_duration_secs = 0;
	while (!solved && (current_duration_secs < allocated_time_secs)) {
		// Setting the allowed time for this optimization loop
		double new_allocated_time_secs = allocated_time_secs - current_duration_secs;
		app_->Options()->SetNumericValue("max_cpu_time", new_allocated_time_secs);
		status = app_->OptimizeTNLP(nlp_ptr_);

		if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level)
			solved = true;
		else if (status == Ipopt::Infeasible_Problem_Detected)
			break;

		// Computing the current time
		clock_t current_time = clock() - started_time;
		current_duration_secs = ((double) current_time) / CLOCKS_PER_SEC;
	}

	if (solved) {
		printf("\n\n*** The problem solved!\n");
		solution_ = ipopt_.getSolution();
		locomotion_trajectory_ = model_->evaluateSolution(solution_);
	} else
		printf("\n\n*** The problem FAILED!\n");

	return solved;
}

} //@namespace solver
} //@namespace dwl
