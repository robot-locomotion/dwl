#include <dwl/model/Constraint.h>


namespace dwl
{

namespace model
{

Constraint::Constraint() : constraint_dimension_(0), is_soft_(false)
{
	state_buffer_.set_capacity(4);
}


Constraint::~Constraint()
{

}


void Constraint::modelFromURDFFile(std::string filename,
								   bool info)
{
	modelFromURDFModel(urdf_model::fileToXml(filename), info);
}


void Constraint::modelFromURDFModel(std::string urdf_model,
									bool info)
{
	// Reseting the floating-base system information given an URDF model
	system_.resetFromURDFModel(urdf_model);

	// Initializing the kinematical and dynamical model from the URDF model
	kinematics_.modelFromURDFModel(urdf_model, info);
	dynamics_.modelFromURDFModel(urdf_model, false);

	// Initializing the information of the specific constraint
	init(info);
}


void Constraint::defineAsSoftConstraint()
{
	is_soft_ = true;
}


void Constraint::defineAsHardConstraint()
{
	is_soft_ = false;
}


void Constraint::init(bool info)
{

}


void Constraint::computeSoft(double& constraint_cost,
							 const WholeBodyState& state)
{
	// Getting the constraint value and bounds
	Eigen::VectorXd constraint, lower_bound, upper_bound;
	compute(constraint, state);

	getBounds(lower_bound, upper_bound);

	// Computing the violation vector
	unsigned int bound_dim = lower_bound.size();
	Eigen::VectorXd violation_vec = Eigen::VectorXd::Zero(bound_dim);
	for (unsigned int i = 0; i < bound_dim; i++) {
		double lower_val = lower_bound(i);
		double upper_val = upper_bound(i);
		double const_val = constraint(i);
		if (lower_val > const_val)
			violation_vec(i) += lower_val - const_val;

		if (upper_val < const_val)
			violation_vec(i) += const_val - upper_val;
	}

	// Computing a weighted quadratic cost of the constraint violation
	double weight = 10000;
	constraint_cost = weight * violation_vec.norm();
}


bool Constraint::isSoftConstraint()
{
	return is_soft_;
}


void Constraint::setLastState(WholeBodyState& last_state)
{
	state_buffer_.push_front(last_state);
}


void Constraint::resetStateBuffer()
{
	unsigned int buffer_size = state_buffer_.size();
	WholeBodyState empty_state(state_buffer_[0].joint_pos.size());
	for (unsigned int i = 0; i < buffer_size; i++)
		state_buffer_.push_back();
}


unsigned int Constraint::getConstraintDimension()
{
	// Getting the constraint dimension given a defined constraint function.
	Eigen::VectorXd bound;
	getBounds(bound, bound);
	constraint_dimension_ = bound.size();

	return constraint_dimension_;
}


std::string Constraint::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
