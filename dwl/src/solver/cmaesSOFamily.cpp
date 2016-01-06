#include <dwl/solver/cmaesSOFamily.h>


namespace dwl
{

namespace solver
{

cmaesSOFamily::cmaesSOFamily() : cmaes_params_(NULL)
{
	name_ = "cmaes family";
}


cmaesSOFamily::~cmaesSOFamily()
{

}


void cmaesSOFamily::setAlgorithm(enum CMAESAlgorithms alg)
{
	switch (alg) {
		case CMAES:
			cmaes_params_->set_algo(CMAES);
			break;
		case IPOP:
			cmaes_params_->set_algo(IPOP);
			break;
		case BIPOP:
			cmaes_params_->set_algo(BIPOP);
			break;
		case ACMAES:
			cmaes_params_->set_algo(ACMAES);
			break;
		case AIPOP:
			cmaes_params_->set_algo(AIPOP);
			break;
		case ABIPOP:
			cmaes_params_->set_algo(ABIPOP);
			break;
		case SEPCMAES:
			cmaes_params_->set_algo(SEPCMAES);
			break;
		case SEPIPOP:
			cmaes_params_->set_algo(SEPIPOP);
			break;
		case SEPBIPOP:
			cmaes_params_->set_algo(SEPBIPOP);
			break;
		case SEPACMAES:
			cmaes_params_->set_algo(SEPACMAES);
			break;
		case SEPAIPOP:
			cmaes_params_->set_algo(SEPAIPOP);
			break;
		case SEPABIPOP:
			cmaes_params_->set_algo(SEPABIPOP);
			break;
		case VDCMA:
			cmaes_params_->set_algo(VDCMA);
			break;
		case VDIPOPCMA:
			cmaes_params_->set_algo(VDIPOPCMA);
			break;
		case VDBIPOPCMA:
			cmaes_params_->set_algo(VDBIPOPCMA);
			break;
		default:
			cmaes_params_->set_algo(CMAES);
			break;
	}
}


bool cmaesSOFamily::init()
{
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
		printf(YELLOW "The hard constraint will not consider in the optimization problem."
				COLOR_RESET);

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
	libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(x_l, x_u, state_dim);



	double sigma = 0.1;
	//int lambda = 100; // offsprings at each generation.
	cmaes_params_ = // -1 for automatically decided lambda, 0 is for random seeding of the internal generator.
			new libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>(x0, sigma,
																						 -1, 0, gp);

	double max_iter = 100;
	cmaes_params_->set_max_iter(max_iter);

	double max_fevals = 1000;
	cmaes_params_->set_max_fevals(max_fevals);

	std::string fplot = "out.dat";
	cmaes_params_->set_fplot(fplot);

	bool with_gradient = false;
	cmaes_params_->set_gradient(with_gradient);

	int elitist = 0;
	cmaes_params_->set_elitism(elitist);




	// Wrapping the fitness function
	using namespace std::placeholders;
	fitness_ = std::bind(&cmaesSOFamily::fitnessFunction, this, _1, _2);

	return true;
}


bool cmaesSOFamily::compute(double allocated_time_secs)
{
	// Setting the warm-point
	model_->getStartingPoint(warm_point_);
	cmaes_params_->set_x0(warm_point_);

	// Computing the solution
	libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness_, *cmaes_params_);
	std::cout << "best solution: " << cmasols << std::endl;
	std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();

	return true;
}


double cmaesSOFamily::fitnessFunction(const double* x,
									  const int& n)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);

	// Numerical evaluation of the cost function
	double obj_value = 0;
	model_->evaluateCosts(obj_value, decision_var);

	return obj_value;
}

} //@namespace solver
} //@namespace dwl
