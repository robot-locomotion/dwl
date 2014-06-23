#ifndef DWL_KinematicConstraints_H
#define DWL_KinematicConstraints_H

#include <planning/Constraint.h>


namespace dwl
{

namespace robot
{

class KinematicConstraints : public planning::Constraint
{
	public:
		/** @brief Constructor function */
		KinematicConstraints();

		/** @brief Destructor function */
		~KinematicConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);


};

} //@namespace hyq

} //@namespace dwl

#endif
