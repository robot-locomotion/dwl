#include <model/FloatingBaseSystem.h>


namespace dwl
{

namespace model
{

FloatingBaseSystem::FloatingBaseSystem(bool full, unsigned int _num_joints) : num_system_joints(0),
		num_floating_joints(6 * full), num_joints(_num_joints),
		AX(full), AY(full), AZ(full), LX(full), LY(full), LZ(full),
		type_of_system(FixedBase), num_end_effectors(0)
{

}


FloatingBaseSystem::~FloatingBaseSystem()
{

}


void FloatingBaseSystem::resetFromURDFFile(std::string filename)
{
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

	resetFromURDFModel(model_xml_string);
}


void FloatingBaseSystem::resetFromURDFModel(std::string urdf_model)
{
	// Getting the RBDL model from URDF model
	RigidBodyDynamics::Addons::URDFReadFromString(urdf_model.c_str(), &rbd_model, false);

	// Getting information about the floating-base joints
	urdf_model::JointID floating_joint_names;
	urdf_model::getJointNames(floating_joint_names, urdf_model, urdf_model::floating);
	num_floating_joints = floating_joint_names.size();

	urdf_model::JointID floating_joint_motions;
	if (num_floating_joints > 0) {
		urdf_model::getFloatingBaseJointMotion(floating_joint_motions, urdf_model);
		for (urdf_model::JointID::iterator jnt_it = floating_joint_motions.begin();
				jnt_it != floating_joint_motions.end(); jnt_it++) {
			std::string joint_name = jnt_it->first;
			unsigned int joint_motion = jnt_it->second;
			rbd::Coords6d joint_motion_coord = rbd::Coords6d(joint_motion);
			unsigned int joint_id = floating_joint_names.find(joint_name)->second;

			// Setting the floating joint information
			FloatingBaseJoint joint(true, joint_id, joint_name);
			setFloatingBaseJoint(joint, joint_motion_coord);
		}
	}

	// Getting the information about the actuated joints
	urdf_model::JointID free_joint_names;
	urdf_model::getJointNames(free_joint_names, urdf_model, urdf_model::free);
	unsigned int num_free_joints = free_joint_names.size();
	num_joints = num_free_joints - num_floating_joints;
	for (urdf_model::JointID::iterator jnt_it = free_joint_names.begin();
			jnt_it != free_joint_names.end(); jnt_it++) {
		std::string joint_name = jnt_it->first;
		unsigned int joint_id = jnt_it->second;

		// Checking if it's a virtual floating-base joint
		if (num_floating_joints > 0) {
			if (floating_joint_names.find(joint_name) == floating_joint_names.end()) {
				// Setting the actuated joint information
				Joint joint(joint_id, joint_name);
				setJoint(joint);
			}
		}
	}

	// Getting the floating-base system information
	num_system_joints = num_floating_joints + num_joints;
	if (isFullyFloatingBase()) {
		if (hasFloatingBaseConstraints())
			type_of_system = ConstrainedFloatingBase;
		else
			type_of_system = FloatingBase;
	} else if (num_floating_joints > 0)
		type_of_system = VirtualFloatingBase;
	else
		type_of_system = FixedBase;

	// Getting the end-effectors information
	urdf_model::getEndEffectors(end_effectors, urdf_model);
	num_end_effectors = end_effectors.size();
}


void FloatingBaseSystem::setFloatingBaseJoint(const FloatingBaseJoint& joint,
											  rbd::Coords6d joint_id)
{
	if (joint_id == rbd::AX)
		AX = joint;
	else if (joint_id == rbd::AY)
		AY = joint;
	else if (joint_id == rbd::AZ)
		AZ = joint;
	else if (joint_id == rbd::LX)
		LX = joint;
	else if (joint_id == rbd::LY)
		LY = joint;
	else
		LZ = joint;
}


void FloatingBaseSystem::setJoint(const Joint& joint)
{
	joints[joint.name] = joint.id;
}


void FloatingBaseSystem::setFloatingBaseConstraint(rbd::Coords6d joint_id)
{
	if (joint_id == rbd::AX)
		AX.constrained = true;
	else if (joint_id == rbd::AY)
		AY.constrained = true;
	else if (joint_id == rbd::AZ)
		AZ.constrained = true;
	else if (joint_id == rbd::LX)
		LX.constrained = true;
	else if (joint_id == rbd::LY)
		LY.constrained = true;
	else
		LZ.constrained = true;
}


void FloatingBaseSystem::setTypeOfDynamicSystem(enum TypeOfSystem _type_of_system)
{
	type_of_system = _type_of_system;
}


void FloatingBaseSystem::setJointDoF(unsigned int _num_joints)
{
	num_joints = _num_joints;
}


RigidBodyDynamics::Model& FloatingBaseSystem::getRBDModel()
{
	return rbd_model;
}


const unsigned int& FloatingBaseSystem::getSystemDoF()
{
	return num_system_joints;
}


const unsigned int& FloatingBaseSystem::getFloatingBaseDoF()
{
	return num_floating_joints;
}


const unsigned int& FloatingBaseSystem::getJointDoF()
{
	return num_joints;
}


const FloatingBaseJoint& FloatingBaseSystem::getFloatingBaseJoint(rbd::Coords6d joint)
{
	if (joint == rbd::AX)
		return AX;
	else if (joint == rbd::AY)
		return AY;
	else if (joint == rbd::AZ)
		return AZ;
	else if (joint == rbd::LX)
		return LX;
	else if (joint == rbd::LY)
		return LY;
	else
		return LZ;
}


unsigned int FloatingBaseSystem::getFloatingBaseJointCoordinate(unsigned int id)
{
	if (AX.active && AX.id == id)
		return rbd::AX;
	else if (AY.active && AY.id == id)
		return rbd::AY;
	else if (AZ.active && AZ.id == id)
		return rbd::AZ;
	else if (LX.active && LX.id == id)
		return rbd::LX;
	else if (LY.active && LY.id == id)
		return rbd::LY;
	else if (LZ.active && LZ.id == id)
		return rbd::LZ;
	else {
		printf(RED "ERROR: the %i id doesn't bellow to floating-base joint\n" COLOR_RESET, id);
		return 0;
	}
}


const urdf_model::JointID& FloatingBaseSystem::getJoints()
{
	return joints;
}


enum TypeOfSystem FloatingBaseSystem::getTypeOfDynamicSystem()
{
	return type_of_system;
}


const unsigned int& FloatingBaseSystem::getNumberOfEndEffectors()
{
	return num_end_effectors;
}


void FloatingBaseSystem::getEndEffectors(urdf_model::LinkSelector& _end_effectors)
{
	_end_effectors = end_effectors;
}


bool FloatingBaseSystem::isFullyFloatingBase()
{
	if (AX.active && AY.active && AZ.active && LX.active && LY.active && LZ.active)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::isVirtualFloatingBaseRobot()
{
	if (type_of_system == VirtualFloatingBase)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::isConstrainedFloatingBaseRobot()
{
	if (type_of_system == ConstrainedFloatingBase)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::hasFloatingBaseConstraints()
{
	if (AX.constrained || AY.constrained || AZ.constrained ||
			LX.constrained || LY.constrained || LZ.constrained)
		return true;
	else
		return false;
}


Eigen::VectorXd FloatingBaseSystem::toGeneralizedJointState(const rbd::Vector6d& base_state,
															const Eigen::VectorXd& joint_state)
{
	// Getting the number of joints
	assert(joint_state.size() == getJointDoF());

	// Note that RBDL defines the floating base state as [linear states, angular states]
	Eigen::VectorXd q;
	if (getTypeOfDynamicSystem() == FloatingBase ||
			getTypeOfDynamicSystem() == ConstrainedFloatingBase) {
		q.resize(6 + getJointDoF());

		rbd::Vector6d _base_state = base_state;
		q << rbd::linearPart(_base_state), rbd::angularPart(_base_state), joint_state;
	} else if (getTypeOfDynamicSystem() == VirtualFloatingBase) {
		unsigned int base_dof = getFloatingBaseDoF();
		q.resize(base_dof + getJointDoF());

		Eigen::VectorXd virtual_base(base_dof);
		if (AX.active)
			virtual_base(AX.id) = base_state(rbd::AX);
		if (AY.active)
			virtual_base(AY.id) = base_state(rbd::AY);
		if (AZ.active)
			virtual_base(AZ.id) = base_state(rbd::AZ);
		if (LX.active)
			virtual_base(LX.id) = base_state(rbd::LX);
		if (LY.active)
			virtual_base(LY.id) = base_state(rbd::LY);
		if (LZ.active)
			virtual_base(LZ.id) = base_state(rbd::LZ);

		q << virtual_base, joint_state;
	} else {
		q.resize(getJointDoF());
		q = joint_state;
	}

	return q;
}


void FloatingBaseSystem::fromGeneralizedJointState(rbd::Vector6d& base_state,
												   Eigen::VectorXd& joint_state,
												   const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint state
	joint_state.resize(getJointDoF());

	// Note that RBDL defines the floating base state as [linear states, angular states]
	if (getTypeOfDynamicSystem() == FloatingBase ||
			getTypeOfDynamicSystem() == ConstrainedFloatingBase) {
		base_state << generalized_state.segment<3>(rbd::LX), generalized_state.segment<3>(rbd::AX);
		joint_state = generalized_state.segment(6, getJointDoF());
	} else if (getTypeOfDynamicSystem() == VirtualFloatingBase) {
		if (AX.active)
			base_state(rbd::AX) = generalized_state(AX.id);
		if (AY.active)
			base_state(rbd::AY) = generalized_state(AY.id);
		if (AZ.active)
			base_state(rbd::AZ) = generalized_state(AZ.id);
		if (LX.active)
			base_state(rbd::LX) = generalized_state(LX.id);
		if (LY.active)
			base_state(rbd::LY) = generalized_state(LY.id);
		if (LZ.active)
			base_state(rbd::LZ) = generalized_state(LZ.id);

		joint_state = generalized_state.segment(getFloatingBaseDoF(), getJointDoF());
	} else {
		base_state = rbd::Vector6d::Zero();
		joint_state = generalized_state;
	}
}


void FloatingBaseSystem::setBranchState(Eigen::VectorXd& new_joint_state,
										const Eigen::VectorXd& branch_state,
										unsigned int body_id)
{
	// Getting the base joint id. Note that the floating-base starts the kinematic-tree
	unsigned int base_id = 0;
	if (isFullyFloatingBase() || isVirtualFloatingBaseRobot())
		base_id = getFloatingBaseDoF();

	// Setting the state values of a specific branch to the joint state
	unsigned int parent_id = body_id;
	if (rbd_model.IsFixedBodyId(body_id)) {
		unsigned int fixed_idx = rbd_model.fixed_body_discriminator;
		parent_id = rbd_model.mFixedBodies[body_id - fixed_idx].mMovableParent;
	}

	// Adding the branch state to the joint state. Two safety checking are done; checking that this
	// branch has at least one joint, and checking the size of the new branch state
	if (parent_id != base_id) {
		unsigned int q_index, num_dof = 0;
		do {
			q_index = rbd_model.mJoints[parent_id].q_index - 1;
			parent_id = rbd_model.lambda[parent_id];
			++num_dof;
		} while (parent_id != base_id);

		assert(branch_state.size() == num_dof);
		new_joint_state.segment(q_index, num_dof) = branch_state;
	}
}


Eigen::VectorXd FloatingBaseSystem::getBranchState(Eigen::VectorXd& joint_state,
												   unsigned int body_id)
{
	unsigned int q_index, num_dof = 0;

	// Getting the base joint id. Note that the floating-base starts the kinematic-tree
	unsigned int base_id = 0;
	if (isFullyFloatingBase() || isVirtualFloatingBaseRobot())
		base_id = getFloatingBaseDoF();

	// Setting the state values of a specific branch to the joint state
	unsigned int parent_id = body_id;
	if (rbd_model.IsFixedBodyId(body_id)) {
		unsigned int fixed_idx = rbd_model.fixed_body_discriminator;
		parent_id = rbd_model.mFixedBodies[body_id - fixed_idx].mMovableParent;
	}

	// Adding the branch state to the joint state. Two safety checking are done; checking that this
	// branch has at least one joint, and checking the size of the new branch state
	if (parent_id != base_id) {
		do {
			q_index = rbd_model.mJoints[parent_id].q_index - 1;
			parent_id = rbd_model.lambda[parent_id];
			++num_dof;
		} while (parent_id != base_id);
	}

	Eigen::VectorXd branch_state(num_dof);
	branch_state = joint_state.segment(q_index, num_dof);

	return branch_state;
}

} //@namespace model
} //@namespace dwl
