#ifndef DWL_KinematicConstraints_H
#define DWL_KinematicConstraints_H

#include <constraint/Constraint.h>


namespace dwl
{

namespace constraint
{

class KinematicConstraints : public constraint::Constraint
{
	public:
		/** @brief Constructor function */
		KinematicConstraints();

		/** @brief Destructor function */
		~KinematicConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);
};

} //@namespace constraint
} //@namespace dwl

#endif
