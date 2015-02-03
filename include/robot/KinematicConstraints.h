#ifndef DWL_KinematicConstraints_H
#define DWL_KinematicConstraints_H

#include <locomotion/Constraint.h>


namespace dwl
{

namespace robot
{

class KinematicConstraints : public locomotion::Constraint
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
