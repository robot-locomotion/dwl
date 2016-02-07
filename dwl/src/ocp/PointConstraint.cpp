#include <dwl/ocp/PointConstraint.h>


namespace dwl
{

namespace ocp
{

PointConstraint::PointConstraint()
{

}


PointConstraint::PointConstraint(Eigen::VectorXd& lower_bound,
								 Eigen::VectorXd& upper_bound) :
										 lower_bound_(lower_bound),
										 upper_bound_(upper_bound)
{

}


PointConstraint::~PointConstraint()
{

}


void PointConstraint::compute(Eigen::VectorXd& constraint,
							  const Eigen::VectorXd& state)
{
	constraint = state;
}


void PointConstraint::getBounds(Eigen::VectorXd& lower_bound,
		   	   	   	   	   	    Eigen::VectorXd& upper_bound)
{
	lower_bound = lower_bound_;
	upper_bound = upper_bound_;
}

} //@namespace ocp
} //@namespace dwl
