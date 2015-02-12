#include <locomotion/Constraint.h>


namespace dwl
{

namespace locomotion
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

} //@namespace locomotion
} //@namespace dwl
