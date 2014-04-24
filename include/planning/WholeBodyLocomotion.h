#ifndef DWL_WholeBodyLocomotion_H
#define DWL_WholeBodyLocomotion_H

#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{


class WholeBodyLocomotion
{

	public:
		/** @brief Constructor function */
		WholeBodyLocomotion();

		/** @brief Destructor function */
		~WholeBodyLocomotion() {}

		void reset(dwl::planning::PlanningOfMotionSequences* planner);

		void addConstraint(planning::Constraint* constraint);

		void removeConstraint(std::string constraint_name);

		void addCost(planning::Cost* cost);

		void removeCost(std::string cost_name);

		bool init();

		bool computePlan();


	private:
		dwl::planning::PlanningOfMotionSequences* planner_;
		bool is_learning_;

	protected:
	
	
}; //@class WholeBodyLocomotion

} //@namespace dwl


#endif
