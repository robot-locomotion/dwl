#include <locomotion/WholeBodyTrajectoryOptimization.h>


namespace dwl
{

namespace locomotion
{

WholeBodyTrajectoryOptimization::WholeBodyTrajectoryOptimization() : solver_(NULL)
{

}


WholeBodyTrajectoryOptimization::~WholeBodyTrajectoryOptimization()
{

}

void WholeBodyTrajectoryOptimization::init(solver::OptimizationSolver* solver)
{
	solver_ = solver;
	solver_->init();
}


void WholeBodyTrajectoryOptimization::addDynamicalSystem(model::DynamicalSystem* system)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().addDynamicalSystem(system);
	else
		printf(RED "FATAL: there was not defined a solver for setting the %s dynamical constraint"
				COLOR_RESET, system->getName().c_str());
}


void WholeBodyTrajectoryOptimization::addConstraint(model::Constraint* constraint)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().addConstraint(constraint);
	else
		printf(RED "FATAL: there was not defined a solver for setting the %s constraint"
				COLOR_RESET, constraint->getName().c_str());
}


void WholeBodyTrajectoryOptimization::addCost(model::Cost* cost)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().addCost(cost);
	else
		printf(RED "FATAL: there is not defined a solver for setting the %s cost"
				COLOR_RESET, cost->getName().c_str());
}


void WholeBodyTrajectoryOptimization::setHorizon(unsigned int horizon)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().setHorizon(horizon);
	else
		printf(RED "FATAL: there is not defined a solver for setting the horizon" COLOR_RESET);
}


void WholeBodyTrajectoryOptimization::setStepIntegrationTime(const double& step_time)
{
	solver_->getOptimizationModel().getDynamicalSystem()->setStepIntegrationTime(step_time);
}


bool WholeBodyTrajectoryOptimization::compute(const LocomotionState& current_state,
											  const LocomotionState& desired_state,
											  double computation_time)
{
	if (solver_ != NULL) {
		// Setting the current state and the starting state for the optimization
		solver_->getOptimizationModel().getDynamicalSystem()->setInitialState(current_state);
		solver_->getOptimizationModel().getDynamicalSystem()->setStartingState(current_state);

		// Setting the desired state to the cost functions
		unsigned int num_cost = solver_->getOptimizationModel().getCosts().size();
		for (unsigned int i = 0; i < num_cost; i++)
			solver_->getOptimizationModel().getCosts()[i]->setDesiredState(desired_state);

		return solver_->compute(computation_time);
	} else {
		printf(RED "FATAL: there was not defined a solver" COLOR_RESET);
		return false;
	}
}


model::DynamicalSystem* WholeBodyTrajectoryOptimization::getDynamicalSystem()
{
	return solver_->getOptimizationModel().getDynamicalSystem();
}


const LocomotionTrajectory& WholeBodyTrajectoryOptimization::getWholeBodyTrajectory()
{
	return solver_->getWholeBodyTrajectory();
}


const LocomotionTrajectory& WholeBodyTrajectoryOptimization::getInterpolatedWholeBodyTrajectory(const double& interpolation_time)
{
	// Deleting old information
	interpolated_trajectory_.clear();

	// Getting the whole-body trajectory
	LocomotionTrajectory trajectory = solver_->getWholeBodyTrajectory();

	// Getting the number of joints and end-effectors
	unsigned int num_joints = getDynamicalSystem()->getFloatingBaseSystem().getJointDoF();
	unsigned int num_contacts = getDynamicalSystem()->getFloatingBaseSystem().getNumberOfEndEffectors();

	// Defining splines //TODO for the time being only cubic interpolation is OK
	std::vector<math::CubicSpline> base_spline, joint_spline, control_spline;
	std::vector<std::vector<math::CubicSpline> > contact_force_spline;
	base_spline.resize(6);
	joint_spline.resize(num_joints);
	control_spline.resize(num_joints);
	contact_force_spline.resize(num_contacts);

	// Computing the interpolation of the whole-body trajectory
	double duration;
	unsigned int horizon = solver_->getOptimizationModel().getHorizon();
	for (unsigned int k = 0; k < horizon - 1; k++) {
		// Adding the starting state
		interpolated_trajectory_.push_back(trajectory[k]);

		// Getting the current starting times
		double starting_time = trajectory[k].time;

		if (getDynamicalSystem()->isFixedStepIntegration())
			duration = getDynamicalSystem()->getFixedStepTime();
		else
			duration = trajectory[k+1].time - trajectory[k].time;

		// Interpolating the current state
		LocomotionState current_state(num_joints);
		math::Spline::Point current_point;
		unsigned int index = floor(duration / interpolation_time);
		for (unsigned int t = 0; t < index; t++) {
			double time = starting_time + t * interpolation_time;

			// Base interpolation
			for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
				if (t == 0) {
					// Initialization of the base motion splines
					math::Spline::Point starting(trajectory[k].base_pos(base_idx),
											 	 trajectory[k].base_vel(base_idx),
												 trajectory[k].base_acc(base_idx));
					math::Spline::Point ending(trajectory[k+1].base_pos(base_idx),
											   trajectory[k+1].base_vel(base_idx),
											   trajectory[k+1].base_acc(base_idx));
					base_spline[base_idx].setBoundary(starting_time, duration, starting, ending);
				} else {
					// Getting and setting the interpolated point
					base_spline[base_idx].getPoint(time, current_point);
					current_state.base_pos(base_idx) = current_point.x;
					current_state.base_vel(base_idx) = current_point.xd;
					current_state.base_acc(base_idx) = current_point.xdd;
				}
			}

			// Joint interpolations
			for (unsigned int joint_idx = 0; joint_idx < num_joints; joint_idx++) {
				if (t == 0) {
					// Initialization of the joint motion splines
					math::Spline::Point motion_starting(trajectory[k].joint_pos(joint_idx),
														trajectory[k].joint_vel(joint_idx),
														trajectory[k].joint_acc(joint_idx));
					math::Spline::Point motion_ending(trajectory[k+1].joint_pos(joint_idx),
													  trajectory[k+1].joint_vel(joint_idx),
													  trajectory[k+1].joint_acc(joint_idx));
					joint_spline[joint_idx].setBoundary(starting_time, duration,
														motion_starting, motion_ending);

					// Initialization of the joint control splines
					math::Spline::Point control_starting(trajectory[k].joint_eff(joint_idx));
					math::Spline::Point control_ending(trajectory[k+1].joint_eff(joint_idx));
					control_spline[joint_idx].setBoundary(starting_time, duration,
														  control_starting, control_ending);
				} else {
					// Getting and setting the joint motion interpolated point
					joint_spline[joint_idx].getPoint(time, current_point);
					current_state.joint_pos(joint_idx) = current_point.x;
					current_state.joint_vel(joint_idx) = current_point.xd;
					current_state.joint_acc(joint_idx) = current_point.xdd;

					// Getting and setting the joint motion interpolated point
					control_spline[joint_idx].getPoint(time, current_point);
					current_state.joint_eff(joint_idx) = current_point.x;
				}
			}

			// Contact interpolation if there are part of the optimization variables
			if (trajectory[k].contacts.size() != 0) {
				for (unsigned int contact_idx = 0; contact_idx < num_contacts; contact_idx++) {
					// Resizing the number of contacts
					current_state.contacts.resize(num_contacts);

					// Resizing the number of coordinate of the spline
					contact_force_spline[contact_idx].resize(3);

					// Computing the contact interpolation per each coordinate (x,y,z)
					for (unsigned int coord_idx = 0; coord_idx < 3; coord_idx++) {
						// Getting the 3d coordinate
						rbd::Coords3d coord = rbd::Coords3d(coord_idx);
						if (t == 0) {
							// Initialization of the contact force splines
							math::Spline::Point force_starting(trajectory[k].contacts[contact_idx].force(coord));
							math::Spline::Point force_ending(trajectory[k+1].contacts[contact_idx].force(coord));
							contact_force_spline[contact_idx][coord_idx].setBoundary(starting_time, duration,
																		  force_starting, force_ending);
						} else {
							// Getting and setting the force interpolated point
							contact_force_spline[contact_idx][coord_idx].getPoint(time, current_point);
							current_state.contacts[contact_idx].force(coord_idx) = current_point.x;
						}
					}
				}
			}

			// Adding the current state
			if (t != 0) {
				// Setting the current time
				current_state.time = time;

				interpolated_trajectory_.push_back(current_state);
			}
		}
	}

	// Adding the ending state
	interpolated_trajectory_.push_back(trajectory[horizon-1]);

	return interpolated_trajectory_;
}


} //@namespace locomotion
} //@namespace dwl
