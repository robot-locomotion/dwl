#ifndef DWL__OCP__CONSTRAINT__IMPL_H
#define DWL__OCP__CONSTRAINT__IMPL_H


namespace dwl
{

namespace ocp
{

template <typename TState>
Constraint<TState>::Constraint() : constraint_dimension_(0), is_soft_(false),
	soft_properties_(SoftConstraintProperties(10000., 0., 0.))
{
	state_buffer_.set_capacity(4);
}


template <typename TState>
Constraint<TState>::~Constraint()
{

}


template <typename TState>
void Constraint<TState>::defineAsSoftConstraint()
{
	is_soft_ = true;
}


template <typename TState>
void Constraint<TState>::defineAsHardConstraint()
{
	is_soft_ = false;
}


template <typename TState>
void Constraint<TState>::init(bool info)
{

}


template <typename TState>
void Constraint<TState>::computeSoft(double& constraint_cost,
									 const TState& state)
{
	// Initialization of the cost value
	constraint_cost = 0.;

	// Getting the constraint value and bounds
	Eigen::VectorXd constraint, lower_bound, upper_bound;
	compute(constraint, state);

	getBounds(lower_bound, upper_bound);

	// Computing the violation vector
	double offset_cost = 0.;
	unsigned int vec_dim = constraint.size();
	Eigen::VectorXd violation_vec = Eigen::VectorXd::Zero(vec_dim);
	for (unsigned int i = 0; i < vec_dim; i++) {
		double lower_val = lower_bound(i) + soft_properties_.threshold;
		double upper_val = upper_bound(i) - soft_properties_.threshold;
		double const_val = constraint(i);
		if (lower_val > const_val) {
			switch (soft_properties_.family) {
			case UNWEIGHTED:
				violation_vec(i) += 1.;
				break;
			case QUADRATIC:
				violation_vec(i) += lower_val - const_val;
				break;
			default:
				violation_vec(i) += lower_val - const_val;
				break;
			}
		}

		if (upper_val < const_val) {
			switch (soft_properties_.family) {
			case UNWEIGHTED:
				violation_vec(i) += 1.;
				break;
			case QUADRATIC:
				violation_vec(i) += const_val - upper_val;
				break;
			default:
				violation_vec(i) += const_val - upper_val;
				break;
			}
		}

		if (upper_bound(i) < const_val || lower_bound(i) > const_val)
			offset_cost = soft_properties_.offset;
	}

	// Computing a weighted quadratic cost of the constraint violation
	constraint_cost = soft_properties_.weight * violation_vec.norm() + offset_cost;
}


template <typename TState>
bool Constraint<TState>::isSoftConstraint()
{
	return is_soft_;
}


template <typename TState>
void Constraint<TState>::setSoftProperties(const SoftConstraintProperties& properties)
{
	soft_properties_ = properties;
}


template <typename TState>
void Constraint<TState>::setLastState(TState& last_state)
{
	state_buffer_.push_front(last_state);
}


template <typename TState>
void Constraint<TState>::resetStateBuffer()
{
	unsigned int buffer_size = state_buffer_.size();
	WholeBodyState empty_state(state_buffer_[0].joint_pos.size());
	for (unsigned int i = 0; i < buffer_size; i++)
		state_buffer_.push_back();
}


template <typename TState>
unsigned int Constraint<TState>::getConstraintDimension()
{
	// Getting the constraint dimension given a defined constraint function.
	Eigen::VectorXd bound;
	getBounds(bound, bound);
	constraint_dimension_ = bound.size();

	return constraint_dimension_;
}


template <typename TState>
std::string Constraint<TState>::getName()
{
	return name_;
}

} //@namespace ocp
} //@namespace dwl



#define Constraint(T) template typename DWL_EXPORTS dwl::ocp::Constraint<T>;
#endif 
