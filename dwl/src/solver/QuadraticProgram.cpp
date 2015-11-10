#include <dwl/solver/QuadraticProgram.h>


namespace dwl
{

namespace solver
{

QuadraticProgram::QuadraticProgram() : initialized_solver_(false), variables_(0), constraints_(0)

{

}


QuadraticProgram::~QuadraticProgram()
{

}

Eigen::VectorXd& QuadraticProgram::getOptimalSolution()
{
	return solution_;
}


unsigned int QuadraticProgram::getNumberOfVariables() const
{
	return variables_;
}


unsigned int QuadraticProgram::getNumberOfConstraints() const
{
	return constraints_;
}

} //@namespace solver
} //@namespace dwl
