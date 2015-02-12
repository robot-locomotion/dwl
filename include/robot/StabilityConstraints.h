#ifndef DWL_StabilityConstraints_H
#define DWL_StabilityConstraints_H

#include <locomotion/Constraint.h>


namespace dwl
{

namespace robot
{

class StabilityConstraints : public locomotion::Constraint
{
	public:
		/** @brief Constructor function */
		StabilityConstraints();

		/** @brief Destructor function */
		~StabilityConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);

};

} //@namespace robot
} //@namespace dwl

#endif
