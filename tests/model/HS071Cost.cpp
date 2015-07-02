#include <model/Cost.h>


namespace dwl
{

namespace model
{

class HS071Cost : public Cost
{
	public:
		HS071Cost() {name_ = "HS071";}
		~HS071Cost() {}

		void compute(double& cost,
					 const LocomotionState& state)
		{
//			assert(n == 4);

			cost = state.base_pos(0) * state.base_pos(3) * (state.base_pos(0) +
					state.base_pos(1) + state.base_pos(2)) + state.base_pos(2);
		}

		void computeGradient(Eigen::VectorXd& gradient,
						 	 const LocomotionState& state)
		{
//			assert(n == 4);

			gradient(0) = state.base_pos(0) * state.base_pos(3) +
					state.base_pos(3) * (state.base_pos(0) + state.base_pos(1) + state.base_pos(2));
			gradient(1) = state.base_pos(0) * state.base_pos(3);
			gradient(2) = state.base_pos(0) * state.base_pos(3) + 1;
			gradient(3) = state.base_pos(0) * (state.base_pos(0) + state.base_pos(1) + state.base_pos(2));
		}
};

} //@namespace model
} //@namespace dwl
