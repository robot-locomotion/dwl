#ifndef DWL_IpoptWrapper_H
#define DWL_IpoptWrapper_H

#include <IpTNLP.hpp>
#include <model/Model.h>


namespace dwl
{

namespace solver
{

/**
 * @class IpoptWrapper
 * @brief C++ interfacing with IPOPT.
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
		 * @param Model* A model consists of cost functions and constraints
		 */
		void reset(model::Model* model);

		/**@name Overloaded from TNLP */
		/**
		 * @brief Method to return some info about the NLP
		 */
		bool get_nlp_info(Index& n, Index& m,
						  Index& nnz_jac_g, Index& nnz_h_lag,
						  IndexStyleEnum& index_style);

		/**
		 * @brief Method to return the bounds for my problem
		 */
		bool get_bounds_info(Index n, Number* x_l, Number* x_u,
							 Index m, Number* g_l, Number* g_u);

		/**
		 * @brief Method to return the starting point for the algorithm
		 */
		bool get_starting_point(Index n, bool init_x, Number* x,
								bool init_z, Number* z_L, Number* z_U,
								Index m, bool init_lambda, Number* lambda);

		/**
		 * @brief Method to return the objective value
		 */
		bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

		/**
		 * @brief Method to return the gradient of the objective
		 */
		bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

		/**
		 * @brief Method to return the constraint residuals
		 */
		bool eval_g(Index n, const Number* x,
				    bool new_x, Index m, Number* g);

		/** Method to return:
		 *   1) The structure of the jacobian (if "values" is NULL)
		 *   2) The values of the jacobian (if "values" is not NULL)
		 */
		bool eval_jac_g(Index n, const Number* x, bool new_x,
					    Index m, Index nele_jac, Index* row_entries, Index* col_entries,
					    Number* values);

		/** Method to return:
		 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
		 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
		 */
		bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
					Index m, const Number* lambda, bool new_lambda,
					Index nele_hess, Index* iRow, Index* jCol, Number* values);
		//@}

		/** @name Solution Methods */
		//@{
		/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
		void finalize_solution(Ipopt::SolverReturn status,
							   Index n, const Number* x, const Number* z_L, const Number* z_U,
							   Index m, const Number* g, const Number* lambda,
							   Number obj_value, const Ipopt::IpoptData* ip_data,
							   Ipopt::IpoptCalculatedQuantities* ip_cq);
		//@}

	private:
		/**@name Methods to block default compiler methods.
		 * The compiler automatically generates the following three methods.
		 *  Since the default compiler implementation is generally not what
		 *  you want (for all but the most simple classes), we usually
		 *  put the declarations of these methods in the private section
		 *  and never implement them. This prevents the compiler from
		 *  implementing an incorrect "default" behavior without us
		 *  knowing. (See Scott Meyers book, "Effective C++")
		 *
		 */
		//@{
		//  IpoptWrapper();
		IpoptWrapper(const IpoptWrapper&);
		IpoptWrapper& operator=(const IpoptWrapper&);
		//@}

		model::Model* model_;
};

} //@namespace solver
} //@namespace dwl


#endif
