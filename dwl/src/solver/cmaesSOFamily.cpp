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


libcmaes::FitFunc fsphere = [](const double *x, const int N)
{
  double val = 0.0;
  for (int i=0;i<N;i++)
    val += x[i]*x[i];
  return val;
};

bool cmaesSOFamily::init()
{
	int dim = 10; // problem dimensions.
	std::vector<double> x0(dim,10.0);
	double sigma = 0.1;
	//int lambda = 100; // offsprings at each generation.
	cmaes_params_ = new libcmaes::CMAParameters<>(x0,sigma);

	double max_iter = 100;
	cmaes_params_->set_max_iter(max_iter);

	double max_fevals = 1000;
	cmaes_params_->set_max_fevals(max_fevals);

	std::string fplot = "out.data";
	cmaes_params_->set_fplot(fplot);

	bool with_gradient = false;
	cmaes_params_->set_gradient(with_gradient);

	int elitist = 0;
	cmaes_params_->set_elitism(elitist);


	libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fsphere, *cmaes_params_);
	std::cout << "best solution: " << cmasols << std::endl;
	std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();

	return true;
}


bool cmaesSOFamily::compute(double allocated_time_secs)
{
	return true;
}


double cmaesSOFamily::fitnessFunction(const double* x,
									  const int n)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);

	// Numerical evaluation of the cost function
	double obj_value = 0;
	model_.evaluateCosts(obj_value, decision_var);

	return obj_value;
}

} //@namespace solver
} //@namespace dwl
