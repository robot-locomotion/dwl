#include <dwl/solver/cmaesSOFamily.h>


namespace dwl
{

namespace solver
{

cmaesSOFamily::cmaesSOFamily()
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

			break;
		case IPOP:

			break;
		case BIPOP:

			break;
		case ACMAES:

			break;
		case AIPOP:

			break;
		case ABIPOP:

			break;
		case SEPCMAES:

			break;
		case SEPIPOP:

			break;
		case SEPBIPOP:

			break;
		case SEPACMAES:

			break;
		case SEPAIPOP:

			break;
		case SEPABIPOP:

			break;
		default:
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
	libcmaes::CMAParameters<> cmaparams(x0,sigma);

	double max_iter = 100;
	cmaparams.set_max_iter(max_iter);

	double max_fevals = 1000;
	cmaparams.set_max_fevals(max_fevals);

	std::string fplot = "out.data";
	cmaparams.set_fplot(fplot);

	bool with_gradient = false;
	cmaparams.set_gradient(with_gradient);

	int elitist = 0;
	cmaparams.set_elitism(elitist);

	//cmaparams.set_algo(BIPOP_CMAES);
	libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fsphere, cmaparams);
	std::cout << "best solution: " << cmasols << std::endl;
	std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();

	return true;
}


bool cmaesSOFamily::compute(double allocated_time_secs)
{
	return true;
}

} //@namespace solver
} //@namespace dwl
