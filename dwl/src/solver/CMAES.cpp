#include <dwl/solver/CMAES.h>


namespace dwl
{

namespace solver
{

CMAES::CMAES()
{

}


CMAES::~CMAES()
{

}

libcmaes::FitFunc fsphere = [](const double *x, const int N)
{
  double val = 0.0;
  for (int i=0;i<N;i++)
    val += x[i]*x[i];
  return val;
};

bool CMAES::init()
{
	int dim = 10; // problem dimensions.
	std::vector<double> x0(dim,10.0);
	double sigma = 0.1;
	//int lambda = 100; // offsprings at each generation.
	libcmaes::CMAParameters<> cmaparams(x0,sigma);
	//cmaparams.set_algo(BIPOP_CMAES);
	libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fsphere, cmaparams);
	std::cout << "best solution: " << cmasols << std::endl;
	std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();

	return true;
}


bool CMAES::compute(double allocated_time_secs)
{
	return true;
}

} //@namespace solver
} //@namespace dwl
