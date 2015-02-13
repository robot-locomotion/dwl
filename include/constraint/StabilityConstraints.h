#ifndef DWL_StabilityConstraints_H
#define DWL_StabilityConstraints_H

#include <constraint/Constraint.h>


namespace dwl
{

namespace constraint
{

class StabilityConstraints : public constraint::Constraint
{
	public:
		/** @brief Constructor function */
		StabilityConstraints();

		/** @brief Destructor function */
		~StabilityConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);

};

} //@namespace constraint
} //@namespace dwl

#endif
