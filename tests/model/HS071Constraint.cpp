#include <model/Constraint.h>


namespace dwl
{

namespace model
{

class HS071Constraint : public Constraint
{
	public:
		HS071Constraint()
		{
			name_ = "HS071";
			is_active_constraint_ = true;
			constraint_dimension_ = 2;
		}

		~HS071Constraint() {}

		void compute(Eigen::VectorXd& constraint,
					 const StateModel& state)
		{
//			  assert(n == 4);
//			  assert(m == 2);
			constraint = Eigen::VectorXd::Zero(constraint_dimension_);
			constraint(0) = state.base_pos(0) * state.base_pos(1) *
					state.base_pos(2) * state.base_pos(3);
			constraint(1) = state.base_pos(0) * state.base_pos(0) +
					state.base_pos(1) * state.base_pos(1) + state.base_pos(2) * state.base_pos(2) +
					state.base_pos(3) * state.base_pos(3);
		}

		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const StateModel& state)
		{
			jacobian = Eigen::MatrixXd::Zero(constraint_dimension_,4);
		    jacobian(0,0) = state.base_pos(1) * state.base_pos(2) * state.base_pos(3);
		    jacobian(0,1) = state.base_pos(0) * state.base_pos(2) * state.base_pos(3);
		    jacobian(0,2) = state.base_pos(0) * state.base_pos(1) * state.base_pos(3);
		    jacobian(0,3) = state.base_pos(0) * state.base_pos(1) * state.base_pos(2);

		    jacobian(1,0) = 2 * state.base_pos(0);
		    jacobian(1,1) = 2 * state.base_pos(1);
		    jacobian(1,2) = 2 * state.base_pos(2);
		    jacobian(1,3) = 2 * state.base_pos(3);
		}
};

} //@namespace model
} //@namespace dwl
