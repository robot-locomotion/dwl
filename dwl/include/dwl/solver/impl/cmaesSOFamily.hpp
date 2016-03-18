#ifndef DWL__SOLVER__CMAESSOFAMILY__IMPL_H
#define DWL__SOLVER__CMAESSOFAMILY__IMPL_H

#include <mutex>
std::mutex fmtx;  // protects fitness function


namespace dwl
{

namespace solver
{

template<typename TScaling>
cmaesSOFamily<TScaling>::cmaesSOFamily() : cmaes_params_(NULL),
		initialized_(false), family_((int) CMAES), sigma_(-1.), lambda_(-1),
		max_iteration_(-1), max_fevals_(-1), elitism_(0),
		max_restarts_(0), multithreading_(false)
{
	name_ = "cmaes family";
}


template<typename TScaling>
cmaesSOFamily<TScaling>::~cmaesSOFamily()
{

}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setFromConfigFile(std::string filename)
{
	std::ifstream fin(filename.c_str());

	// Yaml reader
	dwl::YamlWrapper yaml_reader;

	// Parsing the configuration file
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	for (YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
		// Reading the cmaes namespace
		std::string file_ns;
		it.first() >> file_ns;
		printf("Reading the configuration parameters from the %s namespace\n",
				file_ns.c_str());

		// Getting the cmaes namespace
		const YAML::Node& cmaes_ns = *doc.FindValue(file_ns);

		// Reading and setting up the type of family
		int family;
		if (yaml_reader.read(family, cmaes_ns, "family"))
			setFamily(CMAESFamily(family));

		// Reading the sigma value
		double sigma;
		if (yaml_reader.read(sigma, cmaes_ns, "sigma"))
			setInitialDistribution(sigma);

		// Reading the number of offsprings at each generation
		int lambda;
		if (yaml_reader.read(lambda, cmaes_ns, "lambda"))
			setNumberOfOffsprings(lambda);

		// Reading and setting up the allowed number of iteration
		int max_iter;
		if (yaml_reader.read(max_iter, cmaes_ns, "max_iter"))
			setAllowedNumberofIterations(max_iter);

		// Reading and setting up the allowed number of function evaluations
		int max_fevals;
		if (yaml_reader.read(max_fevals, cmaes_ns, "max_fevals"))
			setAllowedNumberOfFunctionEvalutions(max_fevals);

		// Reading and setting up the type of elitism
		int elitism;
		if (yaml_reader.read(elitism, cmaes_ns, "elitism"))
			setElitism(elitism);

		// Reading the multithreading option
		int max_restarts;
		if (yaml_reader.read(max_restarts, cmaes_ns, "max_restarts"))
			setNumberOfRestarts(max_restarts);
		
		// Reading the multithreading option
		bool multithreading;
		if (yaml_reader.read(multithreading, cmaes_ns, "multithreads"))
			setMultithreading(multithreading);
	}
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setFamily(enum CMAESFamily alg)
{
	family_ = alg;

	// Setting up if the parameters pointer was initialized.
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_algo(family_);
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setAllowedNumberofIterations(int max_iter)
{
	max_iteration_ = max_iter;

	// Setting up if the parameters pointer was initialized.
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_max_iter(max_iteration_);
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setAllowedNumberOfFunctionEvalutions(int max_fevals)
{
	max_fevals_ = max_fevals;

	// Setting up if the parameters pointer was initialized.
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_max_fevals(max_fevals_);
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setInitialDistribution(double sigma)
{
	sigma_ = sigma;

	init(); // Note that this parameter is only set in the init() calls
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setNumberOfOffsprings(int lambda)
{
	lambda_ = lambda;
	
	init(); // Note that this parameter is only set in the init() calls
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setElitism(int elitism)
{
	elitism_ = elitism;

	// Setting up if the parameters pointer was initialized.
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_elitism(elitism_);
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setNumberOfRestarts(int max_restarts)
{
	max_restarts_ = max_restarts;
	
	// Setting up if the parameters pointer was initialized
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_restarts(max_restarts_);
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setMultithreading(bool multithreading)
{
	multithreading_ = multithreading;

	// Setting up if the parameters pointer was initialized.
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_mt_feval(multithreading_);
}


template<typename TScaling>
bool cmaesSOFamily<TScaling>::init()
{
	initialized_ = false;
	
	// Initializing the optimization model
	model_->init(true);

	// Resizing the warm point dimension and getting the warm point
	unsigned int state_dim = model_->getDimensionOfState();
	warm_point_.resize(state_dim);
	model_->getStartingPoint(warm_point_);

	// Converting the warm point to std::vector
	std::vector<double> x0(state_dim);
	for (unsigned int i = 0; i < state_dim; i++)
		x0[i] = warm_point_(i);

	// Safety checking of hard-constraints
	unsigned int constraint_dim = model_->getDimensionOfConstraints();
	if (constraint_dim > 0)
		printf(YELLOW "The hard constraint will not consider in the"
				" optimization problem.\n" COLOR_RESET);

	// Getting the bounds of the optimization problem
	// Eigen interfacing to raw buffers
	double x_l[state_dim], x_u[state_dim];
	double g_l[constraint_dim], g_u[constraint_dim];
	Eigen::Map<Eigen::VectorXd> state_lower_bound(x_l, state_dim);
	Eigen::Map<Eigen::VectorXd> state_upper_bound(x_u, state_dim);
	Eigen::Map<Eigen::VectorXd> constraint_lower_bound(g_l, constraint_dim);
	Eigen::Map<Eigen::VectorXd> constraint_upper_bound(g_u, constraint_dim);

	// Evaluating the bounds. Note that CMA-ES cannot handle hard constraints
	model_->evaluateBounds(state_lower_bound, state_upper_bound,
						   constraint_lower_bound, constraint_upper_bound);

	// Defining the associated bound of the genotype and phenotype
	libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
						TScaling> gp(x_l, x_u, state_dim);

	cmaes_params_ =
			new libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
															TScaling>>(x0, sigma_,
																	   lambda_, 0, gp);

	// Setting the previous parameters values
	initialized_ = true;
	setFamily(CMAESFamily(family_));
	setAllowedNumberofIterations(max_iteration_);
	setAllowedNumberOfFunctionEvalutions(max_fevals_);
	setElitism(elitism_);
	setNumberOfRestarts(max_restarts_);
	setMultithreading(multithreading_);

	std::string fplot = "out.dat";
	cmaes_params_->set_fplot(fplot);

	bool with_gradient = false;
	cmaes_params_->set_gradient(with_gradient);

	// Wrapping the fitness function
	fitness_ = std::bind(&cmaesSOFamily<TScaling>::fitnessFunction,
						 this, std::placeholders::_1, std::placeholders::_2);

	return true;
}


template<typename TScaling>
bool cmaesSOFamily<TScaling>::compute(double allocated_time_secs)
{
	// Setting the warm-point
	model_->getStartingPoint(warm_point_);
	cmaes_params_->set_x0(warm_point_);

	// Computing the solution
	libcmaes::CMASolutions cmasols =
			libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
												TScaling>>(fitness_, *cmaes_params_);
	cmasols.print(std::cout, false, cmaes_params_->get_gp());
	std::cout << std::endl << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";

	// Evaluation of the solution
	Eigen::VectorXd solution =
			cmaes_params_->get_gp().pheno(cmasols.best_candidate().get_x_dvec());
	locomotion_trajectory_ = model_->evaluateSolution(solution);

	return cmasols.run_status();

	return true;
}


template<typename TScaling>
double cmaesSOFamily<TScaling>::fitnessFunction(const double* x,
												const int& n)
{
	// Locking the thread for multi-threading cases
	std::lock_guard<std::mutex> lck(fmtx);

	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);

	// Numerical evaluation of the cost function
	double obj_value = 0;
	model_->evaluateCosts(obj_value, decision_var);

	return obj_value;
}

} //@namespace solver
} //@namespace dwl


#define cmaesSOFamily(T) template typename DWL_EXPORTS dwl::solver::cmaesSOFamily<T>;
#endif 
