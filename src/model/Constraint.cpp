#include <model/Constraint.h>


namespace dwl
{

namespace model
{

Constraint::Constraint() : constraint_dimension_(0)
{

}


Constraint::~Constraint()
{

}


void Constraint::setLastState(LocomotionState& last_state)
{
	last_state_ = last_state;
}


unsigned int Constraint::getConstraintDimension()
{
	// Getting the constraint dimension given a defined constraint function. The constraint
	// dimension will be different to zero once the constraint dimension is defined
	if (constraint_dimension_ == 0) {
		Eigen::VectorXd bound;
		getBounds(bound, bound);
		constraint_dimension_ = bound.size();
	}

	return constraint_dimension_;
}


std::string Constraint::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
