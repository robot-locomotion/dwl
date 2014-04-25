#include <planning/Constraint.h>


namespace dwl
{

namespace planning
{


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
