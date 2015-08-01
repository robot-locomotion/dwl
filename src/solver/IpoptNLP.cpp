#include <solver/IpoptNLP.h>


namespace dwl
{

namespace solver
{

IpoptNLP::IpoptNLP()
{
	name_ = "IpoptNLP";

	// Setting the optimization model to Ipopt wrapper
	ipopt_.setOptimizationModel(&model_);
}


IpoptNLP::~IpoptNLP()
{

}


bool IpoptNLP::init()
{
	// Create a new instance of your NLP
	nlp_ptr_ = &ipopt_;

	// Creating a new instance of IpoptApplication (use a SmartPtr, not raw). We are using the
	// factory, since this allows us to compile this example with an Ipopt Windows DLL
	app_ = IpoptApplicationFactory();
	app_->RethrowNonIpoptException(true);

	// Change some options (do not touch these)
	app_->Options()->SetNumericValue("tol", 1e-7);
	app_->Options()->SetStringValue("mu_strategy", "adaptive");
//	app_->Options()->SetStringValue("output_file", "ipopt.out");
	app_->Options()->SetIntegerValue("max_iter", std::numeric_limits<int>::max());

	// Computing Hessian numerically (do not need to implement)
	app_->Options()->SetStringValue("hessian_approximation", "limited-memory");
//	app_->Options()->SetStringValue("warm_start_init_point", "yes");

	// Initialize the IpoptApplication and process the options
	Ipopt::ApplicationReturnStatus status;
	status = app_->Initialize();
	if (status != Ipopt::Solve_Succeeded) {
		printf("\n\n*** Error during initialization!\n");
		return (int) status;
	}

	return true;
}


bool IpoptNLP::compute(double computation_time)
{
	app_->Options()->SetNumericValue("max_cpu_time", computation_time);

	// Ask Ipopt to solve the problem
	Ipopt::ApplicationReturnStatus status;

	status = app_->OptimizeTNLP(nlp_ptr_);

	if (status == Ipopt::Solve_Succeeded) {
		printf("\n\n*** The problem solved!\n");
	} else {
		printf("\n\n*** The problem FAILED!\n");
	}

	return true;
}

} //@namespace solver
} //@namespace dwl
