#include <dwl/locomotion/WholeBodyTrajectoryOptimization.h>


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

void WholeBodyTrajectoryOptimization::init(solver::OptimizationSolver* solver,
										   std::string config_filename)
{
	solver_ = solver;
	solver_->setOptimizationModel(&oc_model_);
	solver_->init();

	// Setting up the configuration parameters of solver
	if (config_filename != std::string())
		solver_->setFromConfigFile(config_filename);
}


void WholeBodyTrajectoryOptimization::addDynamicalSystem(ocp::DynamicalSystem* system)
{
	oc_model_.addDynamicalSystem(system);
}


void WholeBodyTrajectoryOptimization::removeDynamicalSystem()
{
	oc_model_.removeDynamicalSystem();
}


void WholeBodyTrajectoryOptimization::addConstraint(ocp::Constraint<WholeBodyState>* constraint)
{
	oc_model_.addConstraint(constraint);
}


void WholeBodyTrajectoryOptimization::removeConstraint(std::string name)
{
	oc_model_.removeConstraint(name);
}


void WholeBodyTrajectoryOptimization::addCost(ocp::Cost* cost)
{
	oc_model_.addCost(cost);
}


void WholeBodyTrajectoryOptimization::removeCost(std::string name)
{
	oc_model_.removeCost(name);
}


void WholeBodyTrajectoryOptimization::setHorizon(unsigned int horizon)
{
	oc_model_.setHorizon(horizon);
}


void WholeBodyTrajectoryOptimization::setStepIntegrationTime(const double& step_time)
{
	oc_model_.getDynamicalSystem()->setStepIntegrationTime(step_time);
}


void WholeBodyTrajectoryOptimization::setNominalTrajectory(std::vector<WholeBodyState>& nom_trajectory)
{
	oc_model_.setStartingTrajectory(nom_trajectory);
}


bool WholeBodyTrajectoryOptimization::compute(const WholeBodyState& current_state,
											  const WholeBodyState& desired_state,
											  double computation_time)
{
	// Setting the current state, terminal and the starting state for the optimization
	oc_model_.getDynamicalSystem()->setInitialState(current_state);
	oc_model_.getDynamicalSystem()->setTerminalState(desired_state);

	// Setting the desired state to the cost functions
	unsigned int num_cost = oc_model_.getCosts().size();
	for (unsigned int i = 0; i < num_cost; i++)
		oc_model_.getCosts()[i]->setDesiredState(desired_state);

	return solver_->compute(computation_time);
}


ocp::DynamicalSystem* WholeBodyTrajectoryOptimization::getDynamicalSystem()
{
	return oc_model_.getDynamicalSystem();
}


const WholeBodyTrajectory& WholeBodyTrajectoryOptimization::getWholeBodyTrajectory()
{
	return oc_model_.evaluateSolution(solver_->getSolution());
}


const WholeBodyTrajectory& WholeBodyTrajectoryOptimization::getInterpolatedWholeBodyTrajectory(const double& interpolation_time)
{
	// Deleting old information
	interpolated_trajectory_.clear();

	// Getting the whole-body trajectory
	WholeBodyTrajectory trajectory = getWholeBodyTrajectory();

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
	unsigned int horizon = oc_model_.getHorizon();
	for (unsigned int k = 0; k < horizon; k++) {
		// Adding the starting state
		interpolated_trajectory_.push_back(trajectory[k]);

		// Getting the current starting times
		double starting_time = trajectory[k].time;
		double duration = trajectory[k+1].duration;

		// Interpolating the current state
		WholeBodyState current_state(num_joints);
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

			// Compute the contact information
			// Computing the contact positions
			rbd::BodySelector end_effector_names = getDynamicalSystem()->getFloatingBaseSystem().getEndEffectorNames();
			getDynamicalSystem()->getKinematics().computeForwardKinematics(current_state.contact_pos,
																		   current_state.base_pos,
																		   current_state.joint_pos,
																		   end_effector_names,
																		   dwl::rbd::Linear);
			// Computing the contact velocities
			getDynamicalSystem()->getKinematics().computeVelocity(current_state.contact_vel,
																  current_state.base_pos,
																  current_state.joint_pos,
																  current_state.base_vel,
																  current_state.joint_vel,
																  end_effector_names,
																  dwl::rbd::Linear);
			// Computing the contact accelerations
			getDynamicalSystem()->getKinematics().computeAcceleration(current_state.contact_acc,
																	  current_state.base_pos,
																	  current_state.joint_pos,
																	  current_state.base_vel,
																	  current_state.joint_vel,
																	  current_state.base_acc,
																	  current_state.joint_acc,
																	  end_effector_names,
																	  dwl::rbd::Linear);
			// Computing the contact forces
			getDynamicalSystem()->getDynamics().estimateContactForces(current_state.contact_eff,
																	 current_state.base_pos,
																	 current_state.joint_pos,
																	 current_state.base_vel,
																	 current_state.joint_vel,
																	 current_state.base_acc,
																	 current_state.joint_acc,
																	 current_state.joint_eff,
																	 end_effector_names);

			// Adding the current state
			if (t != 0) {
				// Setting the current time
				current_state.time = time;

				interpolated_trajectory_.push_back(current_state);
			}
		}
	}

	// Adding the ending state
	interpolated_trajectory_.push_back(trajectory[horizon]);

	return interpolated_trajectory_;
}


} //@namespace locomotion
} //@namespace dwl
