#include <model/Constraint.h>
#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

class FullDynamicSystem : public Constraint
{
	public:
		FullDynamicSystem();
		~FullDynamicSystem();

		void compute(Eigen::VectorXd& constraint, const Eigen::Ref<const Eigen::VectorXd>& state);

};

} //@namespace model
} //@namespace dwl
