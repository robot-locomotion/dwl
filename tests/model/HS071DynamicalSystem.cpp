#ifndef DWL__MODEL__HS071_DYNAMICAL_SYSTEM__H
#define DWL__MODEL__HS071_DYNAMICAL_SYSTEM__H

#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

class HS071DynamicalSystem : public DynamicalSystem
{
	public:
		HS071DynamicalSystem()
		{
			name_ = "HS071";
			state_dimension_ = 4;
			locomotion_variables_.position = true; //TODO remove it
			system_.setJointDoF(4);
			system_.setTypeOfDynamicSystem(FixedBase);

			LocomotionState starting_state;
			starting_state.joint_pos.resize(4);
			starting_state.joint_pos(0) = 1.0;
			starting_state.joint_pos(1) = 5.0;
			starting_state.joint_pos(2) = 5.0;
			starting_state.joint_pos(3) = 1.0;
			setInitialState(starting_state);

			LocomotionState lower_state_bound, upper_state_bound;
			lower_state_bound.joint_pos.resize(4);
			lower_state_bound.joint_pos(0) = 1.0;
			lower_state_bound.joint_pos(1) = 1.0;
			lower_state_bound.joint_pos(2) = 1.0;
			lower_state_bound.joint_pos(3) = 1.0;

			upper_state_bound.joint_pos.resize(4);
			upper_state_bound.joint_pos(0) = 5.0;
			upper_state_bound.joint_pos(1) = 5.0;
			upper_state_bound.joint_pos(2) = 5.0;
			upper_state_bound.joint_pos(3) = 5.0;
			setStateBounds(lower_state_bound, upper_state_bound);
		}

		~HS071DynamicalSystem() {}

		void compute(Eigen::VectorXd& constraint,
					 const LocomotionState& state)
		{
			constraint = Eigen::VectorXd::Zero(constraint_dimension_);
			constraint(0) = state.joint_pos(0) * state.joint_pos(1) *
					state.joint_pos(2) * state.joint_pos(3);
			constraint(1) = state.joint_pos(0) * state.joint_pos(0) +
					state.joint_pos(1) * state.joint_pos(1) + state.joint_pos(2) * state.joint_pos(2) +
					state.joint_pos(3) * state.joint_pos(3);
		}

		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound)
		{
			lower_bound = Eigen::VectorXd::Zero(constraint_dimension_);
			lower_bound(0) = 25.0;
			lower_bound(1) = 40.0;

			upper_bound = Eigen::VectorXd::Zero(constraint_dimension_);
			upper_bound(0) = NO_BOUND;
			upper_bound(1) = 40.0;
		}
};

} //@namespace model
} //@namespace dwl

#endif
