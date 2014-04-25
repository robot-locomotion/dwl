#ifndef DWL_HyQ_StabilityConstraints_H
#define DWL_HyQ_StabilityConstraints_H

#include <planning/Constraint.h>


namespace dwl
{

namespace hyq
{

class StabilityConstraints : public planning::Constraint
{
	public:
		/** @brief Constructor function */
		StabilityConstraints();

		/** @brief Destructor function */
		~StabilityConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);


}; //@class StabilityConstraint

} //@namespace hyq

} //@namespace dwl

#endif
