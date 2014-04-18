#ifndef DWL_Constraint_H
#define DWL_Constraint_H

#include <Eigen/Dense>
//#include <iostream>
#include <utils/macros.h>


namespace dwl
{

namespace planning
{

// Active constraints g(x) = 0. Inactive constraints g(x) > 0
class Constraint
{
	public:
		/** @brief Constructor function */
		Constraint() : is_active_constraint_(false) {}

		/** @brief Destructor function */
		virtual ~Constraint() {}

		virtual void get(Eigen::VectorXd& constraint, Eigen::VectorXd state) = 0;

		bool isActive();

		std::string getName();

	protected:
		std::string name_;
		bool is_active_constraint_;
		Eigen::VectorXd state_value_;
		Eigen::VectorXd constraint_value_;


}; //@class Constraint


inline std::string Constraint::getName()
{
	return name_;
}

inline bool Constraint::isActive()
{
	return is_active_constraint_;
}


} //@namespace planning

} //@namespace dwl




#endif
