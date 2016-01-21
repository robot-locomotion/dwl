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

	// Getting the number of lines
	unsigned int num_vertex = polygon.size();
	if (num_vertex == 1)
		num_lines_ = 0;
	else if (num_vertex == 2)
		num_lines_ = 1;
	else
		num_lines_ = num_vertex;


	Eigen::Vector3d extended_point(state.point(rbd::X), state.point(rbd::Y), 1.);
	if (num_lines_ > 2) { // this is a polygon, so it's imposed an inequality
		// Computing the inequality constraints, i.e. imposing the point
		// position inside the support polygon. This constraints can be
		// expressed as P * [x; y; 1]^T >= 0. where the
		// P = [line1; line_2; ... line_n] is polygon matrix. The line is
		// defined by its coefficient and a polygon margin
		Eigen::MatrixXd polygon_mat = Eigen::MatrixXd::Zero(num_lines_, 3);
		math::LineCoeff2d line_coeff;
		for (unsigned int j = 0; j < num_lines_; j++) {
			// Computing the coefficients of the line between two points
			line_coeff = math::lineCoeff(polygon[j], polygon[(j + 1) % num_lines_]); // normalized to use margin

			// Filling the line in the polygon matrix
			polygon_mat(j,0) = line_coeff.p;
			polygon_mat(j,1) = line_coeff.q;
			polygon_mat(j,2) = line_coeff.r - state.margin;
		}
		// Computing as inequality constraint (polygon support)
		constraint = polygon_mat * extended_point;
	} else if (num_lines_ == 1) { // this is a line, so it's imposed an equality
		// Computing the equality constraints, i.e. imposing the point
		// position inside the support line. This constraint can be
		// expressed as p*x + q*y + r = 0. Note that here, we cannot impose
		// a margin.
		// Computing the coefficients of the line between two points
		math::LineCoeff2d line_coeff;
		line_coeff = math::lineCoeff(polygon[0], polygon[1]);

		// Computing as equality constraint (support line)
		double const_value = line_coeff.p * extended_point(rbd::X) +
				line_coeff.q * extended_point(rbd::Y) + line_coeff.r;
		constraint = Eigen::Vector2d(const_value, 0.);
	} else { // this is a point, so it's imposed an equality
		// Computing the equality constraints, i.e. imposing the point
		// position inside the support point. This constraint can be
		double const_value = (extended_point(rbd::X) - polygon[0](rbd::X)) -
				(extended_point(rbd::Y) - polygon[0](rbd::Y));
		constraint = Eigen::Vector2d(const_value, 0.);
	}
}


void SupportPolygonConstraint::getBounds(Eigen::VectorXd& lower_bound,
		   	   	   	   	   	   	   	     Eigen::VectorXd& upper_bound)
{
	if (num_lines_ > 2) { // this is a polygon, so it's imposed an inequality
		lower_bound = Eigen::VectorXd::Zero(num_lines_);
		upper_bound = NO_BOUND * Eigen::VectorXd::Ones(num_lines_);
	} else { // this is a line or point, so it's imposed an equality
		lower_bound = Eigen::VectorXd::Zero(2);
		upper_bound = Eigen::VectorXd::Zero(2);
	}
}

} //@namespace ocp
} //@namespace dwl
