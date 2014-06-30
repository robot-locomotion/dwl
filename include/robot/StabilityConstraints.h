#ifndef DWL_StabilityConstraints_H
#define DWL_StabilityConstraints_H

#include <planning/Constraint.h>


namespace dwl
{

namespace robot
{

class StabilityConstraints : public planning::Constraint
{
	public:
		/** @brief Constructor function */
		StabilityConstraints();

		/** @brief Destructor function */
		~StabilityConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);

};

} //@namespace hyq
} //@namespace dwl

#endif
