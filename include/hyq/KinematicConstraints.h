#ifndef DWL_HyQ_KinematicConstraints_H
#define DWL_HyQ_KinematicConstraints_H

#include <planning/Constraint.h>


namespace dwl
{

namespace hyq
{

class KinematicConstraints : public planning::Constraint
{
	public:
		/** @brief Constructor function */
		KinematicConstraints();

		/** @brief Destructor function */
		~KinematicConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);


}; //@class KinematicConstraint

} //@namespace hyq

} //@namespace dwl

#endif
