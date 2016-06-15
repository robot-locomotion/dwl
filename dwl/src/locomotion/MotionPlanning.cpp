#include <dwl/locomotion/MotionPlanning.h>


namespace dwl
{

namespace locomotion
{

MotionPlanning::MotionPlanning() : terrain_(NULL), robot_(NULL), path_solver_(NULL),
		pose_solver_(NULL), path_computation_time_(std::numeric_limits<double>::max()),
		pose_computation_time_(std::numeric_limits<double>::max())
{

}


MotionPlanning::~MotionPlanning()
{
	delete path_solver_;
	delete pose_solver_;
}


void MotionPlanning::reset(robot::Robot* robot,
						   environment::TerrainMap* terrain)
{
	printf(BLUE "Setting the robot properties in the contact planner \n"
			COLOR_RESET);
	robot_ = robot;

	printf(BLUE "Setting the environment information in the body planner\n"
			COLOR_RESET);
	terrain_ = terrain;

	path_solver_->reset(robot, terrain);
	//pose_solver_->reset(environment); TODO Develop a pose solver
}


void MotionPlanning::reset(solver::SearchTreeSolver* solver)
{
	printf(BLUE "Setting the %s path solver in the %s planner\n" COLOR_RESET,
			solver->getName().c_str(), name_.c_str());
	path_solver_ = solver;
	path_solver_->init();
}


void MotionPlanning::setComputationTime(double computation_time,
										bool path_solver)
{
	if (path_solver) {
		printf("Setting the allowed computation time of the body path solver"
				" to %f \n", computation_time);
		path_computation_time_ = computation_time;
	}
	else {
		printf("Setting the allowed computation time of the body pose solver"
				" to %f \n", computation_time);
		pose_computation_time_ = computation_time;
	}
}

} //@namespace locomotion
} //@namespace dwl
