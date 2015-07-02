#include <model/Model.h>


namespace dwl
{

namespace model
{

class HS071Model : public Model
{
	public:
		HS071Model()
		{
			state_dimension_ = 4;
		}

		~HS071Model() {}

		void convertDecisionVariablesToStateModel(LocomotionState& state_model,
												  const Eigen::VectorXd& decision_var)
		{
			state_model.base_pos(0) = decision_var(0);
			state_model.base_pos(1) = decision_var(1);
			state_model.base_pos(2) = decision_var(2);
			state_model.base_pos(3) = decision_var(3);
		}
};
} //@namespace model
} //@namespace dwl
