#ifndef DWL_StabilityConstraints_H
#define DWL_StabilityConstraints_H

#include <model/Constraint.h>


namespace dwl
{

namespace model
{

class StabilityConstraints : public Constraint
{
	public:
		/** @brief Constructor function */
		StabilityConstraints();

		/** @brief Destructor function */
		~StabilityConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);

};

} //@namespace model
} //@namespace dwl

#endif
