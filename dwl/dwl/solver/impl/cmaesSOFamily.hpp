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
        initialized_(false), print_(false), with_gradient_(false), ftolerance_(1e-12),
		family_((int) CMAES), sigma_(-1.), lambda_(-1), max_iteration_(-1),
		max_fevals_(-1), elitism_(0), max_restarts_(0), multithreading_(false),
		outfile_(false)
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
	// Checking if the solver was initialized before setting it from config file
	bool reinit = false;
	if (initialized_) {
		initialized_ = false;
		reinit = true;
	}

	// Yaml reader
	YamlWrapper yaml_reader(filename);

	// Parsing the configuration file
	std::string cmaes = "cmaes";
	printf(BLUE "Reading the configuration parameters from the %s namespace.\n" COLOR_RESET,
			cmaes.c_str());
	YamlNamespace cmaes_ns = {cmaes};
	YamlNamespace ofile_ns = {cmaes, "output_file"};

	// Reading and setting up the type of ftolerance
	double ftolerance;
	if (yaml_reader.read(ftolerance, "ftolerance", cmaes_ns))
		setFtolerance(ftolerance);

	// Reading and setting up the type of family
	int family;
	if (yaml_reader.read(family, "family", cmaes_ns))
		setFamily(CMAESFamily(family));

	// Reading the sigma value
	double sigma;
	if (yaml_reader.read(sigma, "sigma", cmaes_ns))
		setInitialDistribution(sigma);

	// Reading the number of offsprings at each generation
	int lambda;
	if (yaml_reader.read(lambda, "lambda", cmaes_ns))
		setNumberOfOffsprings(lambda);

	// Reading and setting up the allowed number of iteration
	int max_iter;
	if (yaml_reader.read(max_iter, "max_iter", cmaes_ns))
		setAllowedNumberofIterations(max_iter);

	// Reading and setting up the allowed number of function evaluations
	int max_fevals;
	if (yaml_reader.read(max_fevals, "max_fevals", cmaes_ns))
		setAllowedNumberOfFunctionEvalutions(max_fevals);

	// Reading and setting up the type of elitism
	int elitism;
	if (yaml_reader.read(elitism, "elitism", cmaes_ns))
		setElitism(elitism);

	// Reading the multithreading option
	int max_restarts;
	if (yaml_reader.read(max_restarts, "max_restarts", cmaes_ns))
		setNumberOfRestarts(max_restarts);

	// Reading the gradient injection option
	bool with_grad;
	if (yaml_reader.read(with_grad, "with_gradient", cmaes_ns))
		setGradientInjection(with_grad);

	// Reading the multithreading option
	bool multithreading;
	if (yaml_reader.read(multithreading, "multithreads", cmaes_ns))
		setMultithreading(multithreading);

	// Reading the filename
	bool active;
	if (yaml_reader.read(active, "activate", ofile_ns)) {
		if (active) {
			std::string outfile;
			if (yaml_reader.read(outfile, "filename", ofile_ns)) {
				setOutputFile(outfile);
			}
		}
	}

	// Reading the print option
	bool print;
	if (yaml_reader.read(print, "print", ofile_ns)) {
		setPrintOption(print);
	}

	// Re-initialization of the solver if it was initialized
	if (reinit)
		init();
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setFtolerance(double ftolerance)
{
    ftolerance_ = ftolerance;

    // Setting up if the parameters pointer was initialized.
    // Otherwise it will be initialized when init() is called
    if (initialized_)
        cmaes_params_->set_ftolerance(ftolerance_);
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

	if (initialized_)
		init(); // Note that this parameter is only set in the init() calls
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setNumberOfOffsprings(int lambda)
{
	lambda_ = lambda;
	
	if (initialized_)
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
void cmaesSOFamily<TScaling>::setOutputFile(std::string filename)
{
	outfile_ = true;

	// Setting up if the parameters pointer was initialized
	// Otherwise it will be initialized when init() is called
	if (initialized_)
		cmaes_params_->set_fplot(filename);
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setPrintOption(bool print)
{
	print_ = print;
}


template<typename TScaling>
void cmaesSOFamily<TScaling>::setGradientInjection(bool with_gradient)
{
	with_gradient_ = with_gradient;

	if (initialized_)
		init(); // Note that this parameter is only set in the init() calls
}


template<typename TScaling>
bool cmaesSOFamily<TScaling>::init()
{
	// Initializing the optimization model
	model_->init(true);

	// Resizing the warm point dimension and getting the warm point
	unsigned int state_dim = model_->getDimensionOfState();
	warm_point_.resize(state_dim);
	model_->getStartingPoint(warm_point_.data(), state_dim);

	// Converting the warm point to std::vector
	std::vector<double> x0(state_dim);
	for (unsigned int i = 0; i < state_dim; i++)
		x0[i] = warm_point_(i);

	// Safety checking of hard-constraints
	constraint_dim_ = model_->getDimensionOfConstraints();
	if (constraint_dim_ > 0) {
		printf(BLUE "Info: The hard constraints will be described as soft"
				" constraints.\n" COLOR_RESET);
	}

	// Getting the bounds of the optimization problem
	// Eigen interfacing to raw buffers
	double x_l[state_dim], x_u[state_dim];
	double g_l[constraint_dim_], g_u[constraint_dim_];
	Eigen::Map<Eigen::VectorXd> state_lower_bound(x_l, state_dim);
	Eigen::Map<Eigen::VectorXd> state_upper_bound(x_u, state_dim);
	Eigen::Map<Eigen::VectorXd> constraint_lower_bound(g_l, constraint_dim_);
	Eigen::Map<Eigen::VectorXd> constraint_upper_bound(g_u, constraint_dim_);

	// Evaluating the bounds. Note that CMA-ES cannot handle hard constraints
	model_->evaluateBounds(x_l, state_dim, x_u, state_dim,
						   g_l, constraint_dim_, g_u, constraint_dim_);

	// Defining the associated bound of the genotype and phenotype
	libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
						TScaling> gp(x_l, x_u, state_dim);

	cmaes_params_ =
			new libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
															TScaling>>(x0, sigma_,
																	   lambda_, 0, gp);

	// Setting the previous parameters values
	initialized_ = true;
    setFtolerance(ftolerance_);
	setFamily(CMAESFamily(family_));
	setAllowedNumberofIterations(max_iteration_);
	setAllowedNumberOfFunctionEvalutions(max_fevals_);
	setElitism(elitism_);
	setNumberOfRestarts(max_restarts_);
	setMultithreading(multithreading_);
	if (outfile_)
		cmaes_params_->set_fplot(output_file_);

	// Wrapping the fitness function
	fitness_ = std::bind(&cmaesSOFamily<TScaling>::fitnessFunction,
						 this, std::placeholders::_1, std::placeholders::_2);

	// Wrapping the gradient of the fitness function
	if (with_gradient_) {
		grad_fitness_ = std::bind(&cmaesSOFamily<TScaling>::gradientFitnessFunction,
				 	 	 	 	  this, std::placeholders::_1, std::placeholders::_2);
		cmaes_params_->set_gradient(with_gradient_);
	}

	return true;
}


template<typename TScaling>
bool cmaesSOFamily<TScaling>::compute(double allocated_time_secs)
{
	// Setting the warm-point
	model_->getStartingPoint(warm_point_.data(), warm_point_.size());
	cmaes_params_->set_x0(warm_point_);

	// Computing the solution
	libcmaes::CMASolutions cmasols;
	if (with_gradient_)
		cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
												TScaling>>(fitness_, *cmaes_params_,
														libcmaes::CMAStrategy<libcmaes::CovarianceUpdate,
																			  libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,TScaling>>::_defaultPFunc,
														 					  grad_fitness_);
	else
		cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,
														TScaling>>(fitness_, *cmaes_params_);

	// Prints the solution in the terminal
	if (print_) {
		cmasols.print(std::cout, false, cmaes_params_->get_gp());
		std::cout << std::endl;
		std::cout << "Optimization took ";
		std::cout << cmasols.elapsed_time() / 1000.0 << " seconds\n" << std::endl;
	}

	// Evaluation of the solution
	solution_ =
			cmaes_params_->get_gp().pheno(cmasols.best_candidate().get_x_dvec());

	return cmasols.run_status();
}


template<typename TScaling>
double cmaesSOFamily<TScaling>::fitnessFunction(const double* x,
												const int& n)
{
	// Locking the thread for multi-threading cases
	std::lock_guard<std::mutex> lck(fmtx);

	// Numerical evaluation of the cost function
	double obj_value = 0;
	model_->evaluateCosts(obj_value, x, n);

	if (constraint_dim_ > 0) {
		obj_value += model_->evaluateAsSoftConstraints(x, n);
	}

	return obj_value;
}


template<typename TScaling>
dVec cmaesSOFamily<TScaling>::gradientFitnessFunction(const double *x,
													  const int& n)
{
	dVec gradient(n);

	// Evaluation of the gradient
	model_->evaluateCostGradient(gradient.data(), n, x, n);

	return gradient;
}

} //@namespace solver
} //@namespace dwl


#define cmaesSOFamily(T) template typename DWL_EXPORTS dwl::solver::cmaesSOFamily<T>;
#endif 
