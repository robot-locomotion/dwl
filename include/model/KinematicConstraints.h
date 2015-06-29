#ifndef DWL_KinematicConstraints_H
#define DWL_KinematicConstraints_H

#include <model/Constraint.h>


namespace dwl
{

namespace model
{

class KinematicConstraints : public Constraint
{
	public:
		/** @brief Constructor function */
		KinematicConstraints();

		/** @brief Destructor function */
		~KinematicConstraints();

		void get(Eigen::VectorXd& constraint, Eigen::VectorXd state);
};

} //@namespace model
} //@namespace dwl

#endif
