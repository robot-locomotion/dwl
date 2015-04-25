#include <model/Constraint.h>


namespace dwl
{

namespace model
{

Constraint::Constraint() : is_active_constraint_(false), constraint_dimension_(0)
{

}


Constraint::~Constraint()
{

}


int Constraint::getConstraintDimension()
{
	return constraint_dimension_;
}


std::string Constraint::getName()
{
	return name_;
}


bool Constraint::isActive()
{
	return is_active_constraint_;
}

} //@namespace model
} //@namespace dwl
