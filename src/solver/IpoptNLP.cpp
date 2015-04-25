#include <solver/IpoptNLP.h>


namespace dwl
{

namespace solver
{

IpoptNLP::IpoptNLP()
{

}


IpoptNLP::~IpoptNLP()
{

}


bool IpoptNLP::init()
{
	// Create a new instance of your nlp
	nlp_ptr_ = &ipopt_;//new IpoptWrapper();

	// Create a new instance of IpoptApplication
	//  (use a SmartPtr, not raw)
	// We are using the factory, since this allows us to compile this
	// example with an Ipopt Windows DLL
	app_ = IpoptApplicationFactory();
//	app->RethrowNonIpoptException(true);

	// Change some options (do not touch these)
	app_->Options()->SetNumericValue("tol", 1e-7);
	app_->Options()->SetStringValue("mu_strategy", "adaptive");
	app_->Options()->SetStringValue("output_file", "ipopt.out");
	//app->Options()->SetStringValue("print_user_options","yes");
	//app->Options()->SetStringValue("check_derivatives_for_naninf","yes");
//	app->Options()->SetIntegerValue("max_iter", (int)config_.get<int>("Optim.num_iter")); //set to 1 for debug/comment otherwise
	//print level
//	if  (posOpt->log_->isDebugEnabled())
//		app->Options()->SetIntegerValue("print_level", (int)5);//verbosity The larger this value the more detailed is the output.The valid range for this integer option is 0<print level<12
//	else
//		app->Options()->SetIntegerValue("print_level", (int)0);

	// Compute derivatives numerically
	// Compute Jacobian numerically put return true in eval_grad and eval_jac_g
//	app_->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");
	// compute Hessian numerically (do not need to implement)
//	app_->Options()->SetStringValue("hessian_approximation", "limited-memory");

	//test the derivatives
	//app->Options()->SetStringValue("derivative_test","first-order");//first-order = check jacobians, second-order = check hessian
	//app->Options()->SetStringValue("derivative_test_print_all","no");//no: Print only suspect derivatives, yes: Print all derivatives see the user-provided and estimated derivative values with
	//the relative deviation for each single partial derivative,
	//app->Options()->SetNumericValue("derivative_test_tol", 1); //inf< <0.00001
	//app->Options()->SetNumericValue("derivative_test_perturbation",1e-6);
	//derivative checker
	//* grad_f[          2] = -6.5159999999999991e+02    ~ -6.5559997134793468e+02  [ 6.101e-03]
	//
	//The star (``*'') in the first column indicates that this line corresponds to some partial
	//derivative for which the error tolerance was exceeded. Next, we see which partial derivative
	//is concerned in this output line. For example, in the first line, it is the second component
	//of the objective function gradient (or the third, if the C_STYLE numbering is used, i.e., when
	//counting of indices starts with 0 instead of 1). The first floating point number is the value
	//given by the user code, and the second number (after ``~'') is the finite differences estimation.
	//Finally, the number in square brackets is the relative difference between these two numbers.


	// Initialize the IpoptApplication and process the options
	Ipopt::ApplicationReturnStatus status;
	status = app_->Initialize();
	if (status != Ipopt::Solve_Succeeded) {
		printf("\n\n*** Error during initialization!\n");
		return (int) status;
	}

	return true;
}


bool IpoptNLP::compute()
{
	ipopt_.reset(model_);

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
