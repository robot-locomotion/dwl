#ifndef DWL__SOLVER__IPOPT_WRAPPER__H
#define DWL__SOLVER__IPOPT_WRAPPER__H

#include <IpTNLP.hpp>
#include <model/OptimizationModel.h>


namespace dwl
{

namespace solver
{

/**
 * @class IpoptWrapper
 * @brief C++ interfacing with IPOPT to DWL library. This wrapper calls all the c++ methods which
 * are required to implement constraints, cost functions and them jacobians and gradients,
 * respectively. Constraints and cost functions are encoded in the Model class of DWL, which also
 * can add or remove current constraints or cost functions.
 */
class IpoptWrapper : public Ipopt::TNLP
{
	typedef Ipopt::Index Index;
	typedef Ipopt::Number Number;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRXd;

	public:
		/** @brief Constructor function */
		IpoptWrapper();

		/** @brief Destructor function */
		~IpoptWrapper();

		/**
		 * @brief Defines the model for the optimization
		 * @param model::OptimizationModel* A model consists of cost functions and constraints
		 */
		void reset(model::OptimizationModel* model);

		/**@name Overloaded from TNLP */
		/**
		 * @brief Gets the general information about the NonLinear Program (NLP)
		 * @param Index& Number of decision variables (dimension of $x$)
		 * @param Index& Number of constraint variables (dimension of $g(x)$)
		 * @param Index& Number of nonzero entries in the Jacobian
		 * @param Index& Number of nonzero entries in the Hessian
		 * @param IndexStyleEnum& Numbering style used for row/col entries in the sparse matrix
		 * format (C_STYLE: 0-based, FORTRAN_STYLE: 1-based)
		 */
		bool get_nlp_info(Index& n, Index& m,
						  Index& nnz_jac_g, Index& nnz_h_lag,
						  IndexStyleEnum& index_style);

		/**
		 * @brief Gets the bounds of NLP problem
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param Number* Lower bounds $x^L$ for $x$
		 * @param Number* Upper bounds $x^U$ for $x$
		 * @param Index Number of constraint variables (dimension of $g(x)$)
		 * @param Number* Lower bounds $g^L$ for $x$
		 * @param Number* Upper bounds $g^U$ for $x$
		 */
		bool get_bounds_info(Index n, Number* x_l, Number* x_u,
							 Index m, Number* g_l, Number* g_u);

		/**
		 * @brief Gets the starting point of NLP problem
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param bool If true, this method must provide an initial value for $x$
		 * @param Number* Initial values for the primal variables, $x$
		 * @param bool If true, this method must provide an initial value for the bound multipliers
		 * $z^L$ and $z^U$
		 * @param Number* Initial values for the bound multipliers, $z^L$
		 * @param Number* Iitial values for the bound multipliers, $z^U$
		 * @param Index Number of constraint variables (dimension of $g(x)$)
		 * @param bool If true, this method must provide an initial value for the constraint
		 * multipliers, $\lambda$ .
		 * @param Number* Initial values for the constraint multipliers, $\lambda$
		 */
		bool get_starting_point(Index n, bool init_x, Number* x,
								bool init_z, Number* z_L, Number* z_U,
								Index m, bool init_lambda, Number* lambda);

		/**
		 * @brief Gets the objective (cost function) value
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param Number* Values for the primal variables, $x$, at which $f(x)$ is to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values
		 * in $x$, true otherwise
		 * @param Number& Value of the objective function ($f(x)$).
		 */
		bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

		/**
		 * @brief Gets the gradient of the objective
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param Number* Values for the primal variables, $x$, at which $\nabla f(x)$ is to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values
		 * in $x$, true otherwise
		 * @param Number* Array of values for the gradient of the objective function ($\nabla f(x)$)
		 */
		bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

		/**
		 * @brief Gets the constraint residuals
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param Number* Values for the primal variables, $x$, at which the constraint functions, $g(x)$,
		 * are to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values
		 * in $x$, true otherwise
		 * @param Index Number of constraint variables (dimension of $g(x)$)
		 * @param Number* Array of constraint function values, $g(x)$
		 */
		bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

		/**
		 * @brief This method returns the structure of the Jacobian (if "values" is NULL) or
		 * the values of the Jacobian (if "values" is not NULL)
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param const Number* Values for the primal variables, $x$, at which the constraint Jacobian,
		 * $\nabla g(x)^T$, is to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values
		 * in $x$, true otherwise
		 * @param Index Number of constraint variables (dimension of $g(x)$)
		 * @param Index Number of nonzero elements in the Jacobian (dimension of row_entries,
		 * col_entries, and values)
		 * @param Index* Row indices of entries in the Jacobian of the constraints
		 * @param Index* Column indices of entries in the Jacobian of the constraints
		 * @param Number* Values of the entries in the Jacobian of the constraints
		 */
		bool eval_jac_g(Index n, const Number* x, bool new_x,
					    Index m, Index nele_jac, Index* row_entries, Index* col_entries,
					    Number* values);

		/**
		 * @brief This method returns the structure of the Hessian of the lagrangian (if "values" is NULL)
		 * and the values of the hessian of the lagrangian (if "values" is not NULL)
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param const Number* Values for the primal variables, $x$, at which the Hessian is to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values
		 * in $x$, true otherwise
		 * @param Number Factor in front of the objective term in the Hessian, $\sigma_f$
		 * @param Index Number of constraint variables (dimension of $g(x)$)
		 * @param const Number* Values for the constraint multipliers, $\lambda$ , at which the Hessian
		 * is to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values in lambda,
		 * true otherwise
		 * @param Index Number of nonzero elements in the Hessian (dimension of row_entries, col_entries
		 * and values)
		 * @param Index* Row indices of entries in the Hessian
		 * @param Index* Column indices of entries in the Hessian
		 * @param Number* Values of the entries in the Hessian
		 */
		bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
					Index m, const Number* lambda, bool new_lambda,
					Index nele_hess, Index* row_entries, Index* col_entries, Number* values);

		/**
		 * @brief This method is called by IPOPT after the algorithm has finished (successfully or even
		 * with most errors), so the TNLP can store/write the solution
		 * @param Ipopt::SolverReturn Gives the status of the algorithm as specified in IpAlgTypes.hpp
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param const Number* Values for the primal variables, $x$
		 * @param const Number* Final values for the lower bound multipliers, $z^L_*$
		 * @param const Number* Final values for the upper bound multipliers, $z^U_*$
		 * @param Index Number of constraints in the problem (dimension of $g(x)$)
		 * @param const Number* Final value of the constraint function values, $g(x_*)$
		 * @param const Number* Final values of the constraint multipliers, $\lambda_*$
		 * @param Number Final value of the objective, $f(x_*)$
		 * @param const Ipopt::IpoptData* Ipopt data
		 * @param Ipopt::IpoptCalculatedQuatities* Ipopt calculated quantities
		 */
		void finalize_solution(Ipopt::SolverReturn status,
							   Index n, const Number* x, const Number* z_L, const Number* z_U,
							   Index m, const Number* g, const Number* lambda,
							   Number obj_value, const Ipopt::IpoptData* ip_data,
							   Ipopt::IpoptCalculatedQuantities* ip_cq);


	private:
		/**
		 * @name Methods to block default compiler methods.
		 * @brief The compiler automatically generates the following three methods. Since the default
		 * compiler implementation is generally not what you want (for all but the most simple classes),
		 * we usually put the declarations of these methods in the private section and never implement them.
		 * This prevents the compiler from implementing an incorrect "default" behavior without us knowing
		 * (See Scott Meyers book, "Effective C++")
		 */
		IpoptWrapper(const IpoptWrapper&);
		IpoptWrapper& operator=(const IpoptWrapper&);

		/** @brief Pointer to the defined model of the NLP problem */
		model::OptimizationModel* opt_model_;
};

} //@namespace solver
} //@namespace dwl


#endif
