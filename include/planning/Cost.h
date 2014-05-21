#ifndef DWL_Cost_H
#define DWL_Cost_H

#include <Eigen/Dense>
#include <utils/macros.h>


namespace dwl
{

namespace planning
{

/**
 * @class Cost
 * @brief Abstract class for computing the cost of the planning of motion sequence problem (optimization problem)
 */
class Cost
{
	public:
		/** @brief Constructor function */
		Cost() {}

		/** @brief Destructor function */
		virtual ~Cost() {}

		/**
		 * @brief Abstract method for getting the cost value given a certain state
		 * @param double cost Cost value
		 * @param Eigen::VectorXd state State value
		 */
		virtual void get(double cost, Eigen::VectorXd state) = 0;

		/**
		 * @brief Gets the name of the cost
		 * @return std::string Return the name of the cost
		 */
		std::string getName();

	protected:
		/** @brief Name of the cost */
		std::string name_;

};

} //@namespace planning

} //@namespace dwl

inline std::string dwl::planning::Cost::getName()
{
	return name_;
}


#endif
