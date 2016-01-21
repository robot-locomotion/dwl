#include <dwl/ocp/SupportPolygonConstraint.h>


namespace dwl
{

namespace ocp
{

SupportPolygonConstraint::SupportPolygonConstraint() : num_lines_(10)
{

}


SupportPolygonConstraint::~SupportPolygonConstraint()
{

}


void SupportPolygonConstraint::compute(Eigen::VectorXd& constraint,
									   const PolygonState& state)
{
	// Ordering the polygon vertexes in order to implement the constraints
	std::vector<Eigen::Vector3d> polygon = state.vertexes;
	math::counterClockwiseSort(polygon);
	num_lines_ = polygon.size();

	// Computing the inequality constraints, i.e. imposing the point
	// position inside the support polygon. This constraints can be
	// expressed as P * [x; y; 1]^T >= 0. where the
	// P = [line1; line_2; ... line_n] is polygon matrix. The line is
	// defined by its coefficient and a polygon margin
	Eigen::MatrixXd polygon_mat = Eigen::MatrixXd::Zero(num_lines_, 3);
	math::LineCoeff2d line_coeff;
	for (unsigned int j = 0; j < num_lines_; j++) {
		// Computing the coefficients of the line between two feet
		line_coeff = math::lineCoeff(polygon[j], polygon[(j + 1) % num_lines_]); //I set true to normalize and use stab margin

		// Filling the line in the polygon matrix
		polygon_mat(j,0) = line_coeff.p;
		polygon_mat(j,1) = line_coeff.q;
		polygon_mat(j,2) = line_coeff.r - state.margin;
	}
	Eigen::Vector3d extended_cop(state.point(rbd::X), state.point(rbd::Y), 1.);

	// Computing the constraint
	constraint = polygon_mat * extended_cop;
}


void SupportPolygonConstraint::getBounds(Eigen::VectorXd& lower_bound,
		   	   	   	   	   	   	   	     Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(num_lines_);
	upper_bound = NO_BOUND * Eigen::VectorXd::Ones(num_lines_);
}

} //@namespace ocp
} //@namespace dwl
