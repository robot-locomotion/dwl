#include <model/ComplementaryConstraint.h>


namespace dwl
{

namespace model
{

ComplementaryConstraint::ComplementaryConstraint() : complementary_dimension_(0)
{

}


ComplementaryConstraint::~ComplementaryConstraint()
{

}


void ComplementaryConstraint::init(std::string urdf_model,
								   bool info)
{

}


void ComplementaryConstraint::compute(Eigen::VectorXd& constraint,
									  const LocomotionState& state)
{
	// Computing the first and second constraints
	Eigen::VectorXd first_constraint, second_constraint;
	computeFirstComplement(first_constraint, state);
	computeSecondComplement(second_constraint, state);

	// Resizing the constraint vector
	constraint.resize(2 * complementary_dimension_ + 1);

	// Adding the complement constraints
	constraint.segment(0, complementary_dimension_) = first_constraint;
	constraint.segment(complementary_dimension_, complementary_dimension_) = second_constraint;

	// Computing the inner product of the complementary constraints
	constraint.segment<1>(2 * complementary_dimension_) =
			first_constraint.transpose() * second_constraint;
}


void ComplementaryConstraint::getBounds(Eigen::VectorXd& lower_bound,
										Eigen::VectorXd& upper_bound)
{
	// Resizing the bounds
	lower_bound.resize(2 * complementary_dimension_ + 1);
	upper_bound.resize(2 * complementary_dimension_ + 1);

	// Computing the lower and upper bound of the first and second constraints
	lower_bound.segment(0, 2 * complementary_dimension_) =
			Eigen::VectorXd::Zero(2 * complementary_dimension_);
	upper_bound.segment(0, 2 * complementary_dimension_) =
			NO_BOUND * Eigen::VectorXd::Ones(2 * complementary_dimension_);

	// Computing the inner product bounds
	double relaxation_parameter = 0.5;
	lower_bound.segment<1>(2 * complementary_dimension_) = Eigen::VectorXd::Zero(1);
	upper_bound.segment<1>(2 * complementary_dimension_) =
			relaxation_parameter * Eigen::VectorXd::Ones(1);
}

} //@namespace model
} //@namespace dwl
