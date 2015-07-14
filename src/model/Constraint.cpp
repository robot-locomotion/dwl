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


unsigned int Constraint::getConstraintDimension()
{
	return constraint_dimension_;
}


std::string Constraint::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
