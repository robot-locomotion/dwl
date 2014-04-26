#ifndef DWL_PlanningOfMotionSequences_H
#define DWL_PlanningOfMotionSequences_H

#include <planning/Solver.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>
#include <pthread.h>


namespace dwl
{

namespace planning
{

struct Contact
{
	int end_effector;
	Eigen::Vector3d position;
};

struct BodyPose
{
	Eigen::Vector3d position;
	Eigen::Vector3d orientation;
};


class PlanningOfMotionSequences
{
	public:
		/** @brief Constructor function */
		PlanningOfMotionSequences();

		/** @brief Destructor function */
		virtual ~PlanningOfMotionSequences() {}

		/**
		@brief Function to specify the settings of all variables within the Planning of Motion Sequences problem (solver algorithmn)
		@param dwl::planning::Solver *solver	Pointer to the solver of the motion planning algorithm
		 */
		void reset(Solver* solver);

		void addConstraint(Constraint* constraint);

		void removeConstraint(std::string constraint_name);

		void addCost(Cost* cost);

		void removeCost(std::string cost_name);

		bool initPlan(std::vector<double> start, std::vector<double> goal);

		virtual bool init(std::vector<double> start, std::vector<double> goal) = 0;

		bool computePlan();

		virtual bool compute() = 0;

		void changeGoal(std::vector<double> goal);

		std::string getName();


	private:
		bool is_settep_solver_;
		bool is_initialized_planning_;

		/** @brief Initial state of the robot */
		std::vector<double> initial_state_;

		/** @brief Goal state of the robot */
		std::vector<double> goal_state_;

		pthread_mutex_t planning_lock_;

	protected:
		std::string name_;
		Solver* solver_;
		std::vector<Contact> contacts_sequence_;
		std::vector<BodyPose> pose_trajectory_;
};

} //@namespace planning

} //@namespace dwl


#endif
