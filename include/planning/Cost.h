#ifndef DWL_Cost_H
#define DWL_Cost_H

#include <Eigen/Dense>
#include <utils/macros.h>


namespace dwl
{

namespace planning
{

class Cost
{
	public:
		/** @brief Constructor function */
		Cost() {}

		/** @brief Destructor function */
		virtual ~Cost() {}

		virtual void get(double cost, Eigen::VectorXd state) = 0;

		std::string getName();

	protected:
		std::string name_;

};

} //@namespace planning

} //@namespace dwl

inline std::string dwl::planning::Cost::getName()
{
	return name_;
}


#endif
