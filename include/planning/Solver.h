#ifndef DWL_Solver_H
#define DWL_Solver_H

#include <planning/Constraint.h>
#include <planning/Cost.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>
#include <pthread.h>


namespace dwl
{

namespace planning
{

class Solver
{
	public:
		/** @brief Constructor function */
		Solver();

		/** @brief Destructor function */
		virtual ~Solver();

		virtual bool init() = 0;
		virtual bool compute(Eigen::MatrixXd& solution) = 0;

		void addConstraint(Constraint* constraint);
		void removeConstraint(std::string constraint_name);
		void addCost(Cost* cost);
		void removeCost(std::string cost_name);

		std::string getName();

	private:

	protected:
		std::string name_;
		bool is_added_active_constraint_, is_added_inactive_constraint_, is_added_cost_;
		std::vector<Constraint*> active_constraints_;
		std::vector<Constraint*> inactive_constraints_;
		std::vector<Cost*> costs_;
		pthread_mutex_t solver_lock_;
};

} //@namespace planning

} //@namespace dwl


#endif
