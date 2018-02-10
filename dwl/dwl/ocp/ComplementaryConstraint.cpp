#include <dwl/ocp/ComplementaryConstraint.h>


namespace dwl
{

namespace ocp
{

ComplementaryConstraint::ComplementaryConstraint() : complementary_dimension_(0)
{

}


ComplementaryConstraint::~ComplementaryConstraint()
{

}


void ComplementaryConstraint::reset(model::FloatingBaseSystem& fbs,
									model::WholeBodyKinematics& wkin,
				        			model::WholeBodyDynamics& wdyn)
{
	// Creating the floating-base system and whole-body kinematics shared pointers
	fbs_ = std::make_shared<model::FloatingBaseSystem>(fbs);
	wkin_ = std::make_shared<model::WholeBodyKinematics>(wkin);
	wdyn_ = std::make_shared<model::WholeBodyDynamics>(wdyn);

	// Setting the whole-body dynamics from floating-base model
	wdyn_->reset(fbs, wkin);
}


void ComplementaryConstraint::init(std::string urdf_model,
								   bool info)
{

}


void ComplementaryConstraint::compute(Eigen::VectorXd& constraint,
									  const WholeBodyState& state)
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
	constraint(2 * complementary_dimension_) = first_constraint.transpose() * second_constraint;
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
	lower_bound(2 * complementary_dimension_) = -NO_BOUND;
	upper_bound(2 * complementary_dimension_) = 0.0;
}

} //@namespace ocp
} //@namespace dwl
