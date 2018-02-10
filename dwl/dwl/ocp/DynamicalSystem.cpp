#include <dwl/ocp/DynamicalSystem.h>


namespace dwl
{

namespace ocp
{

DynamicalSystem::DynamicalSystem() : state_dimension_(0), terminal_constraint_dimension_(0),
		system_variables_(false), integration_method_(Fixed), step_time_(0.1),
		is_full_trajectory_optimization_(false)
{

}


DynamicalSystem::~DynamicalSystem()
{

}


void DynamicalSystem::reset(model::FloatingBaseSystem& fbs,
							model::WholeBodyKinematics& wkin,
				        	model::WholeBodyDynamics& wdyn)
{
	// Creating the floating-base system and whole-body kinematics shared pointers
	fbs_ = std::make_shared<model::FloatingBaseSystem>(fbs);
	wkin_ = std::make_shared<model::WholeBodyKinematics>(wkin);
	wdyn_ = std::make_shared<model::WholeBodyDynamics>(wdyn);

	// Setting the whole-body dynamics from floating-base model
	wdyn_->reset(fbs, wkin);
}


void DynamicalSystem::init(bool info)
{
	// Computing the state dimension of the dynamical system constraint
	computeStateDimension();

	// Setting initial conditions
	initialConditions();

	// Reading and setting the joint limits
	jointLimitsFromURDF();

	// Initializing the dynamical system
	initDynamicalSystem();

	if (info) {
		printf("The state dimension is %i\n", state_dimension_);
		printf("The full DOF of floating-base system is %i\n", fbs_->getSystemDoF());
		printf("The joint DOF of floating-base system is %i\n", fbs_->getJointDoF());
	}
}


void DynamicalSystem::initDynamicalSystem()
{

}


void DynamicalSystem::jointLimitsFromURDF()
{
	// Initializing the joint limits
	unsigned int num_joints = fbs_->getJointDoF();
	Eigen::VectorXd lower_joint_pos, upper_joint_pos, joint_vel, joint_eff;
	lower_joint_pos = -NO_BOUND * Eigen::VectorXd::Ones(num_joints);
	upper_joint_pos = NO_BOUND * Eigen::VectorXd::Ones(num_joints);
	joint_vel = NO_BOUND * Eigen::VectorXd::Ones(num_joints);
	joint_eff = NO_BOUND * Eigen::VectorXd::Ones(num_joints);

	// Reading the joint limits
	dwl::urdf_model::JointLimits joint_limits = fbs_->getJointLimits();
	dwl::urdf_model::JointID joint_ids = fbs_->getJoints();
	for (dwl::urdf_model::JointLimits::iterator jnt_it = joint_limits.begin();
			jnt_it != joint_limits.end(); ++jnt_it) {
		std::string joint_name = jnt_it->first;
		urdf::JointLimits joint_limits = jnt_it->second;
		unsigned int joint_id = joint_ids.find(joint_name)->second;

		lower_joint_pos(joint_id) = joint_limits.lower;
		upper_joint_pos(joint_id) = joint_limits.upper;
		joint_vel(joint_id) = joint_limits.velocity;
		joint_eff(joint_id) = joint_limits.effort;
	}
}


void DynamicalSystem::compute(Eigen::VectorXd& constraint,
							  const WholeBodyState& state)
{
	// Evaluating the numerical integration
	Eigen::VectorXd time_constraint;
	numericalIntegration(time_constraint, state);

	// Computing the dynamical constraint
	Eigen::VectorXd dynamical_constraint;
	computeDynamicalConstraint(dynamical_constraint, state);

	// Adding both constraints
	unsigned int dynamical_dim = dynamical_constraint.size();
	constraint.resize(fbs_->getSystemDoF() + dynamical_dim);
	constraint.segment(0, fbs_->getSystemDoF()) = time_constraint;
	constraint.segment(fbs_->getSystemDoF(), dynamical_dim) = dynamical_constraint;
}


void DynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
												 const WholeBodyState& state)
{
	printf(RED "FATAL: the dynamical constraint was not implemented\n" COLOR_RESET);
	exit(EXIT_FAILURE);
}


void DynamicalSystem::computeTerminalConstraint(Eigen::VectorXd& constraint,
												const WholeBodyState& state)
{
	constraint.resize(fbs_->getFloatingBaseDoF());

	// Computing the position error
	Eigen::VectorXd position_error =
			fbs_->toGeneralizedJointState(terminal_state_.base_pos, terminal_state_.joint_pos) -
			fbs_->toGeneralizedJointState(state.base_pos, state.joint_pos);

	// Adding the terminal constraint
	constraint = position_error.head(fbs_->getFloatingBaseDoF());
}


void DynamicalSystem::numericalIntegration(Eigen::VectorXd& constraint,
										   const WholeBodyState& state)
{
	// Resizing the constraint vector
	constraint.resize(fbs_->getSystemDoF());

	// Transcription of the constrained inverse dynamic equation using Euler-backward integration.
	// This integration method adds numerical stability
	Eigen::VectorXd base_int = state_buffer_[0].base_pos - state.base_pos + state.duration * state.base_vel;
	Eigen::VectorXd joint_int = state_buffer_[0].joint_pos - state.joint_pos + state.duration * state.joint_vel;

	// Adding the time integration constraint
	constraint = fbs_->toGeneralizedJointState(base_int, joint_int);
}


void DynamicalSystem::getBounds(Eigen::VectorXd& lower_bound,
								Eigen::VectorXd& upper_bound)
{
	Eigen::VectorXd time_bound = Eigen::VectorXd::Zero(fbs_->getSystemDoF());

	// Getting the dynamical bounds
	Eigen::VectorXd dynamical_lower_bound, dynamical_upper_bound;
	getDynamicalBounds(dynamical_lower_bound, dynamical_upper_bound);

	// Adding both bounds
	unsigned int dynamical_dim = dynamical_lower_bound.size();
	lower_bound.resize(fbs_->getSystemDoF() + dynamical_dim);
	upper_bound.resize(fbs_->getSystemDoF() + dynamical_dim);
	lower_bound.segment(0, fbs_->getSystemDoF()) = time_bound;
	lower_bound.segment(fbs_->getSystemDoF(), dynamical_dim) = dynamical_lower_bound;
	upper_bound.segment(0, fbs_->getSystemDoF()) = time_bound;
	upper_bound.segment(fbs_->getSystemDoF(), dynamical_dim) = dynamical_upper_bound;
}


void DynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
										 Eigen::VectorXd& upper_bound)
{
	printf(RED "FATAL: the dynamical bounds was not implemented\n" COLOR_RESET);
	exit(EXIT_FAILURE);
}


void DynamicalSystem::getTerminalBounds(Eigen::VectorXd& lower_bound,
										Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(fbs_->getFloatingBaseDoF());
	upper_bound = Eigen::VectorXd::Zero(fbs_->getFloatingBaseDoF());
}


void DynamicalSystem::setStepIntegrationMethod(StepIntegrationMethod method)
{
	integration_method_ = method;
	if (integration_method_ == Fixed)
		system_variables_.time = false;
	else
		system_variables_.time = true;

	// Setting the state dimension
	computeStateDimension();
}


void DynamicalSystem::setStepIntegrationTime(const double& step_time)
{
	step_time_ = step_time;
}


std::shared_ptr<model::FloatingBaseSystem> DynamicalSystem::getFloatingBaseSystem()
{
	return fbs_;
}


std::shared_ptr<model::WholeBodyKinematics> DynamicalSystem::getKinematics()
{
	return wkin_;
}


std::shared_ptr<model::WholeBodyDynamics> DynamicalSystem::getDynamics()
{
	return wdyn_;
}


void DynamicalSystem::setStateBounds(const WholeBodyState& lower_bound,
									 const WholeBodyState& upper_bound)
{
	lower_state_bound_ = lower_bound;
	upper_state_bound_ = upper_bound;
}


void DynamicalSystem::setInitialState(const WholeBodyState& initial_state)
{
	initial_state_ = initial_state;
}


void DynamicalSystem::setTerminalState(const WholeBodyState& terminal_state)
{
	terminal_state_ = terminal_state;
}


void DynamicalSystem::getStateBounds(WholeBodyState& lower_bound,
									 WholeBodyState& upper_bound)
{
	lower_bound = lower_state_bound_;
	upper_bound = upper_state_bound_;
}


const WholeBodyState& DynamicalSystem::getInitialState()
{
	return initial_state_;
}


const WholeBodyState& DynamicalSystem::getTerminalState()
{
	return terminal_state_;
}


unsigned int DynamicalSystem::getDimensionOfState()
{
	return state_dimension_;
}


unsigned int DynamicalSystem::getTerminalConstraintDimension()
{
	return terminal_constraint_dimension_;
}


const double& DynamicalSystem::getFixedStepTime()
{
	return step_time_;
}


void DynamicalSystem::setFullTrajectoryOptimization()
{
	is_full_trajectory_optimization_ = true;
}


void DynamicalSystem::toWholeBodyState(WholeBodyState& system_state,
									   const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint dimensions
	system_state.joint_pos = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	system_state.joint_vel = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	system_state.joint_acc = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	system_state.joint_eff = Eigen::VectorXd::Zero(fbs_->getJointDoF());

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (system_variables_.time) {
		system_state.duration = generalized_state(idx);
		++idx;
	}
	if (system_variables_.position) {
		fbs_->fromGeneralizedJointState(system_state.base_pos,
										  system_state.joint_pos,
										  (Eigen::VectorXd) generalized_state.segment(idx, fbs_->getSystemDoF()));
		idx += fbs_->getSystemDoF();
	}
	if (system_variables_.velocity) {
		fbs_->fromGeneralizedJointState(system_state.base_vel,
										  system_state.joint_vel,
										  (Eigen::VectorXd) generalized_state.segment(idx, fbs_->getSystemDoF()));
		idx += fbs_->getSystemDoF();
	}
	if (system_variables_.acceleration) {
		fbs_->fromGeneralizedJointState(system_state.base_acc,
										  system_state.joint_acc,
										  (Eigen::VectorXd) generalized_state.segment(idx, fbs_->getSystemDoF()));
		idx += fbs_->getSystemDoF();
	}
	if (system_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		model::FloatingBaseSystem fake_system(false, fbs_->getJointDoF());
		fake_system.fromGeneralizedJointState(system_state.base_eff,
											  system_state.joint_eff,
											  (Eigen::VectorXd) generalized_state.segment(idx, fbs_->getJointDoF()));
		idx += fbs_->getJointDoF();
	}
	if (system_variables_.contact_pos || system_variables_.contact_vel ||
			system_variables_.contact_acc || system_variables_.contact_for) {
		urdf_model::LinkID contact_links = fbs_->getEndEffectors();
		for (urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
				contact_it != contact_links.end(); contact_it++) {
			std::string name = contact_it->first;

			if (system_variables_.contact_pos) {
				system_state.contact_pos[name] = generalized_state.segment<3>(idx);
				idx += 3;
			}
			if (system_variables_.contact_vel) {
				system_state.contact_vel[name] = generalized_state.segment<3>(idx);
				idx += 3;
			}
			if (system_variables_.contact_acc) {
				system_state.contact_acc[name] = generalized_state.segment<3>(idx);
				idx += 3;
			}
			if (system_variables_.contact_for) {
				system_state.contact_eff[name] << 0, 0, 0, generalized_state.segment<3>(idx);
				idx += 3;
			}
		}
	}
}


void DynamicalSystem::fromWholeBodyState(Eigen::VectorXd& generalized_state,
										 const WholeBodyState& system_state)
{
	// Resizing the generalized state vector
	generalized_state.resize(state_dimension_);

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (system_variables_.time) {
		generalized_state(idx) = system_state.duration;
		++idx;
	}
	if (system_variables_.position) {
		generalized_state.segment(idx, fbs_->getSystemDoF()) =
				fbs_->toGeneralizedJointState(system_state.base_pos,
												system_state.joint_pos);
		idx += fbs_->getSystemDoF();
	}
	if (system_variables_.velocity) {
		generalized_state.segment(idx, fbs_->getSystemDoF()) =
				fbs_->toGeneralizedJointState(system_state.base_vel,
												system_state.joint_vel);
		idx += fbs_->getSystemDoF();
	}
	if (system_variables_.acceleration) {
		generalized_state.segment(idx, fbs_->getSystemDoF()) =
				fbs_->toGeneralizedJointState(system_state.base_acc,
												system_state.joint_acc);
		idx += fbs_->getSystemDoF();
	}
	if (system_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		model::FloatingBaseSystem fake_system(false, fbs_->getJointDoF());
		generalized_state.segment(idx, fbs_->getJointDoF()) =
				fake_system.toGeneralizedJointState(system_state.base_eff,
											 	 	system_state.joint_eff);
		idx += fbs_->getJointDoF();
	}
	if (system_variables_.contact_pos || system_variables_.contact_vel ||
			system_variables_.contact_acc || system_variables_.contact_for) {
		urdf_model::LinkID contact_links = fbs_->getEndEffectors();
		for (urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
				contact_it != contact_links.end(); contact_it++) {
			std::string name = contact_it->first;

			if (system_variables_.contact_pos) {
				generalized_state.segment<3>(idx) = system_state.contact_pos.at(name);
				idx += 3;
			}
			if (system_variables_.contact_vel) {
				generalized_state.segment<3>(idx) = system_state.contact_vel.at(name);
				idx += 3;
			}
			if (system_variables_.contact_acc) {
				generalized_state.segment<3>(idx) = system_state.contact_acc.at(name);
				idx += 3;
			}
			if (system_variables_.contact_for) {
				generalized_state.segment<3>(idx) = system_state.contact_eff.at(name).segment<3>(rbd::LZ);
				idx += 3;
			}
		}
	}
}


bool DynamicalSystem::isFixedStepIntegration()
{
	return !system_variables_.time;
}


bool DynamicalSystem::isFullTrajectoryOptimization()
{
	return is_full_trajectory_optimization_;
}


void DynamicalSystem::computeStateDimension()
{
	// Computing the state dimension give the locomotion variables
	state_dimension_ = system_variables_.time + (system_variables_.position +
			system_variables_.velocity + system_variables_.acceleration) * fbs_->getSystemDoF()
			+ system_variables_.effort * fbs_->getJointDoF() + 3 *
			(system_variables_.contact_pos + system_variables_.contact_vel +
					system_variables_.contact_acc + system_variables_.contact_for) *
					fbs_->getNumberOfEndEffectors();
}


void DynamicalSystem::initialConditions()
{
	// Setting the terminal constraint dimension
	terminal_constraint_dimension_ = fbs_->getFloatingBaseDoF();

	// No-bound limit by default
	lower_state_bound_.duration = 0.05;
	lower_state_bound_.base_pos = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_pos = -NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	lower_state_bound_.base_vel = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_vel = -NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	lower_state_bound_.base_acc = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_acc = -NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	lower_state_bound_.base_eff = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_eff = -NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	upper_state_bound_.duration = 0.5;//NO_BOUND;
	upper_state_bound_.base_pos = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_pos = NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	upper_state_bound_.base_vel = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_vel = NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	upper_state_bound_.base_acc = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_acc = NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	upper_state_bound_.base_eff = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_eff = NO_BOUND * Eigen::VectorXd::Ones(fbs_->getJointDoF());
	urdf_model::LinkID contact_links = fbs_->getEndEffectors();
	for (urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
			contact_it != contact_links.end(); contact_it++) {
		std::string name = contact_it->first;

		lower_state_bound_.contact_pos[name] = -NO_BOUND * Eigen::Vector3d::Ones();
		lower_state_bound_.contact_vel[name] = -NO_BOUND * Eigen::Vector3d::Ones();
		lower_state_bound_.contact_acc[name] = -NO_BOUND * Eigen::Vector3d::Ones();
		lower_state_bound_.contact_eff[name] = -NO_BOUND * rbd::Vector6d::Ones();
		upper_state_bound_.contact_pos[name] = NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contact_vel[name] = NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contact_acc[name] = NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contact_eff[name] = NO_BOUND * rbd::Vector6d::Ones();
	}

	// Initial state
	initial_state_.joint_pos = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	initial_state_.joint_vel = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	initial_state_.joint_acc = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	initial_state_.joint_eff = Eigen::VectorXd::Zero(fbs_->getJointDoF());
	for (urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
			contact_it != contact_links.end(); contact_it++) {
		std::string name = contact_it->first;

		initial_state_.contact_pos[name] = Eigen::Vector3d::Zero();
		initial_state_.contact_vel[name] = Eigen::Vector3d::Zero();
		initial_state_.contact_acc[name] = Eigen::Vector3d::Zero();
		initial_state_.contact_eff[name] = rbd::Vector6d::Zero();
	}
}

} //@namespace ocp
} //@namespace dwl
