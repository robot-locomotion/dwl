#include <planning/Constraint.h>


namespace dwl
{

namespace planning
{

Constraint::Constraint() : is_active_constraint_(false)
{

}


Constraint::~Constraint()
{

}


std::string Constraint::getName()
{
	return name_;
}

bool Constraint::isActive()
{
	return is_active_constraint_;
}


} //@namespace planning
} //@namespace dwl
