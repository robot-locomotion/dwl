#include <dwl/solver/QuadraticProgram.h>


namespace dwl
{

namespace solver
{

QuadraticProgram::QuadraticProgram() : initialized_solver_(false), horizon_(0), variables_(0),
		constraints_(0)

{

}


QuadraticProgram::~QuadraticProgram()
{

}

void QuadraticProgram::setHorizon(int horizon)
{
	horizon_ = horizon;
}


void QuadraticProgram::setNumberOfVariables(int variables)
{
	variables_ = variables;
}


int QuadraticProgram::getNumberOfVariables() const
{
	return variables_;
}


int QuadraticProgram::getNumberOfConstraints() const
{
	return constraints_;
}

} //@namespace solver
} //@namespace dwl
