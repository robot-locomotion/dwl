#ifndef DWL_PlanningOfMotionSequences_H
#define DWL_PlanningOfMotionSequences_H

#include <planning/Solver.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>


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

		void addCost(Cost* cost);

		virtual bool init(std::vector<double> start, std::vector<double> goal) = 0;

		virtual bool compute() = 0;


		void setGoal(std::vector<double> goal);

		std::string getName();


	private:
		/** @brief Initial state of the robot */
		std::vector<double> initial_state_;

		/** @brief Goal state of the robot */
		std::vector<double> goal_state_;


	protected:
		std::string name_;
		Solver* solver_;
		std::vector<Contact> contacts_sequence_;
		std::vector<BodyPose> pose_trajectory_;
};

} //@namespace planning

} //@namespace dwl


inline std::string dwl::planning::PlanningOfMotionSequences::getName()
{
	return name_;
}


#endif
