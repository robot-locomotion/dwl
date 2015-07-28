#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

DynamicalSystem::DynamicalSystem() : system_(NULL), state_dimension_(0), num_endeffectors_(1),
		system_dof_(0), joint_dof_(0), locomotion_variables_(false)
{

}


DynamicalSystem::~DynamicalSystem()
{

}


void DynamicalSystem::modelFromURDFFile(std::string filename,
										struct rbd::FloatingBaseSystem* system,
										bool info)
{
	// Initializing the dynamic model from the filename
	dynamics_.modelFromURDFFile(filename, system, info);
	system_ = system;

	// Setting initial conditions
	initialConditions();

	// Reading the file
	std::ifstream model_file(filename.c_str());
	if (!model_file) {
		std::cerr << "Error opening file '" << filename << "'." << std::endl;
		abort();
	}

	// Reserving memory for the contents of the file
	std::string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
			std::istreambuf_iterator<char>());
	model_file.close();

	// Parsing the URDF-XML
	boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(model_xml_string);

	// Reading the joint limits
	jointLimitsFromURDF(urdf_model);

	if (info) {
		printf("The state dimension is %i\n", state_dimension_);
		printf("The full DOF of floating-base system is %i\n", system_dof_);
		printf("The joint DOF of floating-base system is %i\n", joint_dof_);
	}
}


void DynamicalSystem::modelFromURDFModel(std::string _urdf_model,
										 struct rbd::FloatingBaseSystem* system,
										 bool info)
{
	// Initializing the dynamic model from the URDF model
	dynamics_.modelFromURDFModel(_urdf_model, system, info);
	system_ = system;

	// Setting initial conditions
	initialConditions();

	// Parsing the URDF-XML
	boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(_urdf_model);

	// Reading and setting the joint limits
	jointLimitsFromURDF(urdf_model);

	if (info) {
		printf("The state dimension is %i\n", state_dimension_);
		printf("The full DOF of floating-base system is %i\n", system_dof_);
		printf("The joint DOF of floating-base system is %i\n", joint_dof_);
	}
}


void DynamicalSystem::jointLimitsFromURDF(boost::shared_ptr<urdf::ModelInterface> model)
{
	// Reading and setting the joint limits
	boost::shared_ptr<urdf::Link>& root = model->links_[model->getRoot()->name];
	boost::shared_ptr<urdf::Link> current_link = root;
	unsigned virtual_joints_idx = 0, joint_idx = 0;
	while (current_link->child_joints.size() > 0) {
		boost::shared_ptr<urdf::Joint> current_joint = current_link->child_joints[0];
		current_link = model->links_[current_joint->child_link_name];
		if (current_joint->type == urdf::Joint::PRISMATIC ||
				current_joint->type == urdf::Joint::REVOLUTE) {
			if (system_->getTypeOfDynamicSystem() == rbd::VirtualFloatingBase) {
				if (system_->getFloatingBaseDOF() != virtual_joints_idx) {
						virtual_joints_idx++;
				} else {
					lower_state_bound_.joint_pos(joint_idx) = current_joint->limits->lower;
					upper_state_bound_.joint_pos(joint_idx) = current_joint->limits->upper;
					lower_state_bound_.joint_vel(joint_idx) = -current_joint->limits->velocity;
					upper_state_bound_.joint_vel(joint_idx) = current_joint->limits->velocity;
					lower_state_bound_.joint_eff(joint_idx) = -current_joint->limits->effort;
					upper_state_bound_.joint_eff(joint_idx) = current_joint->limits->effort;
					joint_idx++;
				}
			}
		}
	}
}


void DynamicalSystem::setFloatingBaseSystem(rbd::FloatingBaseSystem* system)
{
	system_ = system;

	// Getting the dof of the system
	system_dof_ = system_->getSystemDOF();
	joint_dof_ = system_->getJointDOF();
}


void DynamicalSystem::setStartingState(const LocomotionState& starting_state)
{
	starting_state_ = starting_state;
}


void DynamicalSystem::setStateBounds(const LocomotionState& lower_bound,
									 const LocomotionState& upper_bound)
{
	lower_state_bound_ = lower_bound;
	upper_state_bound_ = upper_bound;
}


void DynamicalSystem::getStartingState(LocomotionState& starting_state)
{
	starting_state = starting_state_;
}


void DynamicalSystem::getStateBounds(LocomotionState& lower_bound,
									 LocomotionState& upper_bound)
{
	lower_bound = lower_state_bound_;
	upper_bound = upper_state_bound_;
}


unsigned int DynamicalSystem::getDimensionOfState()
{
	return state_dimension_;
}


unsigned int DynamicalSystem::getNumberOfEndEffectors()
{
	return num_endeffectors_;
}


void DynamicalSystem::toLocomotionState(LocomotionState& locomotion_state,
										const Eigen::VectorXd& generalized_state)
//TODO add other locomotion variables
{
	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		locomotion_state.time = generalized_state(idx);
		++idx;
	}
	if (locomotion_variables_.position) {
		rbd::fromGeneralizedJointState(locomotion_state.base_pos,
									   locomotion_state.joint_pos,
									   (Eigen::VectorXd) generalized_state.segment(idx, system_dof_),
									   system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.velocity) {
		rbd::fromGeneralizedJointState(locomotion_state.base_vel,
									   locomotion_state.joint_vel,
									   (Eigen::VectorXd) generalized_state.segment(idx, system_dof_),
									   system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.acceleration) {
		rbd::fromGeneralizedJointState(locomotion_state.base_acc,
									   locomotion_state.joint_acc,
									   (Eigen::VectorXd) generalized_state.segment(idx, system_dof_),
									   system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		rbd::FloatingBaseSystem fake_system(false, joint_dof_);
		rbd::fromGeneralizedJointState(locomotion_state.base_eff,
									   locomotion_state.joint_eff,
									   (Eigen::VectorXd) generalized_state.segment(idx, joint_dof_),
									   &fake_system);
		idx += joint_dof_;
	}
}


void DynamicalSystem::fromLocomotionState(Eigen::VectorXd& generalized_state,
										  const LocomotionState& locomotion_state)
//TODO add other locomotion variables
{
	// Resizing the generalized state vector
	generalized_state.resize(state_dimension_);

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		generalized_state(idx) = locomotion_state.time;
	}
	if (locomotion_variables_.position) {
		generalized_state.segment(idx, system_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_pos,
											 locomotion_state.joint_pos,
											 system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.velocity) {
		generalized_state.segment(idx, system_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_vel,
											 locomotion_state.joint_vel,
											 system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.acceleration) {
		generalized_state.segment(idx, system_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_acc,
											 locomotion_state.joint_acc,
											 system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		rbd::FloatingBaseSystem fake_system(false, joint_dof_);
		generalized_state.segment(idx, joint_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_eff,
											 locomotion_state.joint_eff,
											 &fake_system);
		idx += joint_dof_;
	}
}


void DynamicalSystem::initialConditions()
{
	// Computing the state dimension give the locomotion variables
	state_dimension_ = (locomotion_variables_.position + locomotion_variables_.velocity
			+ locomotion_variables_.acceleration) * system_->getSystemDOF()
			+ locomotion_variables_.effort * system_->getJointDOF();

	constraint_dimension_ = getConstraintDimension();

	// Getting the dof of the system
	system_dof_ = system_->getSystemDOF();
	joint_dof_ = system_->getJointDOF();

	// No-bound limit by default
	lower_state_bound_.base_pos = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_pos = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	lower_state_bound_.base_vel = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_vel = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	lower_state_bound_.base_acc = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_acc = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	lower_state_bound_.base_eff = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_eff = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_pos = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_pos = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_vel = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_vel = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_acc = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_acc = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_eff = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_eff = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);

	// Starting state
	starting_state_.joint_pos = Eigen::VectorXd::Zero(joint_dof_);
	starting_state_.joint_vel = Eigen::VectorXd::Zero(joint_dof_);
	starting_state_.joint_acc = Eigen::VectorXd::Zero(joint_dof_);
	starting_state_.joint_eff = Eigen::VectorXd::Zero(joint_dof_);
}

} //@namespace model
} //@namespace dwl
