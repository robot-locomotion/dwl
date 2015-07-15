#ifndef DWL__MODEL__STABILITY_CONSTRAINT__H
#define DWL__MODEL__STABILITY_CONSTRAINT__H

#include <model/Constraint.h>


namespace dwl
{

namespace model
{

class StabilityConstraint : public Constraint
{
	public:
		/** @brief Constructor function */
		StabilityConstraint();

		/** @brief Destructor function */
		~StabilityConstraint();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);

};

} //@namespace model
} //@namespace dwl

#endif
