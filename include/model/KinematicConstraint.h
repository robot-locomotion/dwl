#ifndef DWL__MODEL__KINEMATIC_CONSTRAINT__H
#define DWL__MODEL__KINEMATIC_CONSTRAINT__H

#include <model/Constraint.h>


namespace dwl
{

namespace model
{

class KinematicConstraint : public Constraint
{
	public:
		/** @brief Constructor function */
		KinematicConstraint();

		/** @brief Destructor function */
		~KinematicConstraint();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);
};

} //@namespace model
} //@namespace dwl

#endif
