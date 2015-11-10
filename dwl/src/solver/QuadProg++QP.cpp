#include <dwl/solver/QuadProg++QP.h>


namespace dwl
{

namespace solver
{

QuadProgQP::QuadProgQP()
{

}


QuadProgQP::~QuadProgQP()
{

}


bool QuadProgQP::init(unsigned int num_variables,
					  unsigned int num_constraints)
{
	return true;
}


bool QuadProgQP::compute(const Eigen::MatrixXd& hessian,
						 const Eigen::VectorXd& gradient,
						 const Eigen::MatrixXd& constraint_mat,
						 const Eigen::VectorXd& lower_bound,
						 const Eigen::VectorXd& upper_bound,
						 const Eigen::VectorXd& lower_constraint,
						 const Eigen::VectorXd& upper_constraint,
						 double cputime)
{
	// Getting the number of equality and inequality constraints
	unsigned int num_constraints = lower_constraint.size();
	unsigned int num_eq_constraints = 0;
	unsigned int num_ineq_constraints = 0;
	for (unsigned int i = 0; i < num_constraints; i++) {
		if ((upper_constraint - lower_constraint).isZero())
			++num_eq_constraints;
		else
			++num_ineq_constraints;
	}

	// Getting the number of equality and inequality bounds
	unsigned int num_bounds = lower_bound.size();
	unsigned int num_eq_bounds = 0;
	unsigned int num_ineq_bounds = 0;
	for (unsigned int i = 0; i < num_constraints; i++) {
		if ((upper_bound - lower_bound).isZero())
			++num_eq_bounds;
		else
			++num_ineq_bounds;
	}

	// Computing the constraint in/equality matrix and in/equality bound
	Eigen::MatrixXd eq_constraint_mat = Eigen::MatrixXd::Zero(num_eq_constraints + num_eq_bounds,
															  num_eq_constraints + num_eq_bounds);
	Eigen::MatrixXd ineq_constraint_mat = Eigen::MatrixXd::Zero(num_ineq_constraints + num_ineq_bounds,
																num_ineq_constraints + num_ineq_bounds);
	Eigen::VectorXd eq_bound = Eigen::VectorXd::Zero(num_eq_constraints);
	Eigen::VectorXd ineq_bound = Eigen::VectorXd::Zero(num_ineq_constraints);
	unsigned int eq_idx = 0;
	unsigned int ineq_idx = 0;
	for (unsigned int i = 0; i < num_constraints; i++) {
		if ((upper_constraint - lower_constraint).isZero()) {
			// Converting lbA = Ax to Ax -lbA = 0
			eq_constraint_mat.row(eq_idx) = constraint_mat.row(i);
			eq_bound(eq_idx) = -lower_constraint(i);
			++eq_idx;
		} else {
			// Converting lbA <= Ax <= ubA to Ax -lbA >= 0 and -Ax + ubA >= 0
			// Setting the equality bound component
			ineq_constraint_mat.row(ineq_idx) = constraint_mat.row(i);
			ineq_bound(ineq_idx) = -lower_constraint(i);
			++ineq_idx;

			// Setting the inequality bound component
			ineq_constraint_mat.row(ineq_idx) = -constraint_mat.row(i);
			ineq_bound(ineq_idx) = upper_constraint(i);
			++ineq_idx;
		}
	}
	for (unsigned int i = 0; i < num_bounds; i++) {
		Eigen::VectorXd identity_row = Eigen::VectorXd::Zero(num_bounds);
		identity_row(i) = 1;
		if ((upper_bound - lower_bound).isZero()) {
			// Converting lb = x to Ix - lb = 0
			eq_constraint_mat.row(eq_idx) = identity_row;
			eq_bound(eq_idx) = -lower_bound(i);
			++eq_idx;
		} else {
			// Converting lb <= x <= up to Ix - lb >= 0 and -Ix + ub >= 0
			// Setting the equality bound component
			ineq_constraint_mat.row(ineq_idx) = identity_row;
			ineq_bound(ineq_idx) = -lower_bound(i);
			++ineq_idx;

			// Setting the inequality bound component
			ineq_constraint_mat.row(ineq_idx) = -identity_row;
			ineq_bound(ineq_idx) = upper_bound(i);
			++ineq_idx;
		}
	}

	Eigen::MatrixXd copy_hessian = hessian;
	Eigen::VectorXd copy_gradient = gradient;
	solve_quadprog(copy_hessian, copy_gradient,
				   eq_constraint_mat, eq_bound,
				   ineq_constraint_mat, ineq_bound,
				   solution_);

	return true;
}

} //@namespace solver
} //@namespace dwl
