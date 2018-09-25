#include <dwl/model/FloatingBaseSystem.h>


namespace dwl
{

namespace model
{

FloatingBaseSystem::FloatingBaseSystem():
		data_ptr_(NULL), root_joint_(FREE_FLYER),
		n_base_joints_(0), n_joints_(0),
		num_end_effectors_(0), num_feet_(0), grav_acc_(0.)
{

}


FloatingBaseSystem::~FloatingBaseSystem()
{

}


void FloatingBaseSystem::resetFromURDFFile(const std::string& urdf_file,
										   const std::string& system_file)
{
	resetFromURDFModel(dwl::urdf::fileToXml(urdf_file), system_file);
}


void FloatingBaseSystem::resetFromURDFModel(const std::string& urdf_model,
											const std::string& system_file)
{
	// Yaml reader
	YamlWrapper yaml_reader(system_file);

	// Parsing the configuration file
	YamlNamespace robot_ns = {"robot"};

	// Reading the root joint
	int root;
	if (!yaml_reader.read(root, "root_joint", robot_ns)) {
		printf("Error: the root joint was not found\n");
		return;
	}
	root_joint_ = (RootJoint) root;

	// Getting the Pinocchio model from URDF
	urdf_ = urdf_model;
	yarf_ = system_file;

	// Clear content
	joints_.clear();
	joint_names_.clear();
	bodies_.clear();
	body_names_.clear();
	frames_.clear();
	frame_names_.clear();

	// Defining the root joint
	bool floating = false;
	if (root_joint_ == FREE_FLYER) {
		se3::urdf::buildModelFromXML(urdf_, se3::JointModelFreeFlyer(), model_);
		floating = true;
	} else if (root_joint_ == PLANAR) {
		se3::urdf::buildModelFromXML(urdf_, se3::JointModelPlanar(), model_);
		floating = true;
	} else if (root_joint_ == NO)
		se3::urdf::buildModelFromXML(urdf_, model_);
	else {
		se3::urdf::buildModelFromXML(urdf_, se3::JointModelFreeFlyer(), model_);
		floating = true;
	}
	data_ptr_ = new se3::Data(model_);
		
	// Getting the number of joints
	if (floating) {
		floating_joint_name_ = model_.names[1];
		n_base_joints_ = 1;
		n_joints_ = model_.njoints - 2;
	} else {
		n_base_joints_ = 0;
		n_joints_ = model_.njoints - 1;	
	}

	// Resetting the dimension of the configuration and tangent vectors
	q_ = Eigen::VectorXd::Zero(model_.nq);
	v_ = Eigen::VectorXd::Zero(model_.nv);

	for (int f = 1; f < model_.nframes; ++f) {
		se3::Frame frame = model_.frames[f];
		unsigned int fparent = (unsigned int) frame.parent;

		// Getting the joint id and name		
		if (frame.type == se3::FrameType::JOINT &&
				frame.name != "root_joint") {
			if (floating)
				joints_[frame.name] = fparent - 2;
			else
				joints_[frame.name] = fparent - 1;
		}

		// Getting the information of the body names and ids
		if (frame.type == se3::FrameType::BODY) {
			bodies_[frame.name] = fparent;
			if (floating && fparent == 1)
				floating_body_name_ = frame.name;
		}

		// Getting the frame information
		frames_[frame.name] = f;
		frame_names_.push_back(frame.name);
	}

	// Getting the joint name list
	for (ElementId::const_iterator it = joints_.begin();
			it != joints_.end(); ++it) {
		std::string name = it->first;
		joint_names_.push_back(name);
	}

	// Getting the body name list
	for (ElementId::const_iterator it = bodies_.begin();
			it != bodies_.end(); ++it) {
		std::string name = it->first;
		body_names_.push_back(name);
	}


	// // Getting the end-effectors information
	dwl::urdf::getEndEffectorNames(end_effectors_, urdf_model);

	// Getting the end-effector name list
	for (ElementId::const_iterator ee_it = end_effectors_.begin();
			ee_it != end_effectors_.end(); ++ee_it) {
		std::string name = ee_it->first;
		end_effector_names_.push_back(name);
	}

	// Resetting the system description
	default_joint_pos_ = Eigen::VectorXd::Zero(n_joints_);
	if (!system_file.empty())
		resetSystemDescription(system_file);
	
	// Setting up the default posture in the pinocchio model
	dwl::SE3 origin;
	Eigen::VectorXd def_posture = toConfigurationState(origin,
													   default_joint_pos_);
	model_.neutralConfiguration = def_posture;


	// Defining the number of end-effectors
	num_end_effectors_ = end_effectors_.size();

	if (num_feet_ == 0) {
		printf(YELLOW "Warning: setting up all the end-effectors are feet\n"
				COLOR_RESET);
		num_feet_ = num_end_effectors_;
		foot_names_ = end_effector_names_;
		feet_ = end_effectors_;
	}

	// Getting gravity information
	grav_acc_ = model_.gravity.linear().norm();
	grav_dir_ = model_.gravity.linear() / grav_acc_;
}


void FloatingBaseSystem::resetSystemDescription(const std::string& filename)
{
	// Yaml reader
	YamlWrapper yaml_reader(filename);

	// Parsing the configuration file
	std::string robot = "robot";
	YamlNamespace robot_ns = {robot};
	YamlNamespace pose_ns = {robot, "default_pose"};

	// Reading and setting up the foot names
	if (yaml_reader.read(foot_names_, "feet", robot_ns)) {
		// Ordering the foot names
		std::sort(foot_names_.begin(),
				  foot_names_.end(),
				  std::bind(&FloatingBaseSystem::compareString,
				  			this, std::placeholders::_1, std::placeholders::_2)); 

		// Getting the number of foot
		num_feet_ = foot_names_.size();

		// Adding to the feet to the end-effector lists if it doesn't exist
		for (unsigned int i = 0; i < foot_names_.size(); ++i) {
			std::string name = foot_names_[i];
			if (end_effectors_.count(name) == 0) {
				unsigned int id = end_effectors_.size() + 1;
				end_effectors_[name] = id;
				end_effector_names_.push_back(name);
			}
		}
	}

	// Reading the default posture of the robot
	default_joint_pos_ = Eigen::VectorXd::Zero(n_joints_);
	for (unsigned int j = 0; j < n_joints_; ++j) {
		std::string name = joint_names_[j];

		double joint_pos = 0.;
		if (yaml_reader.read(joint_pos, name, pose_ns)) {
			default_joint_pos_(j) = joint_pos;
		}
	}
}


const std::string& FloatingBaseSystem::getURDFModel() const
{
	return urdf_;
}


const std::string& FloatingBaseSystem::getYARFModel() const
{
	return yarf_;
}


se3::Model& FloatingBaseSystem::getModel()
{
	return model_;
}


se3::Data& FloatingBaseSystem::getData()
{
	return *data_ptr_;
}


unsigned int FloatingBaseSystem::getConfigurationDim()
{
	return model_.nq;
}


unsigned int FloatingBaseSystem::getTangentDim()
{
	return model_.nv;
}


const std::string& FloatingBaseSystem::getFloatingBaseName() const
{
	return floating_body_name_;
}


const unsigned int& FloatingBaseSystem::getJointDoF() const
{
	return n_joints_;
}


const unsigned int& FloatingBaseSystem::getJointId(const std::string& name) const
{
	ElementId::const_iterator it = joints_.find(name);
	if (it != joints_.end())
		return it->second;
	else
		return UNDEFINED_ID;
}


const std::string& FloatingBaseSystem::getJointName(const unsigned int& id) const
{
	return joint_names_[id];
}


const ElementList& FloatingBaseSystem::getJointList() const
{
	return joint_names_;
}


const ElementId& FloatingBaseSystem::getJoints() const
{
	return joints_;
}


Eigen::VectorXd FloatingBaseSystem::getLowerLimits()
{
	return model_.lowerPositionLimit.bottomRows(n_joints_);
}


double FloatingBaseSystem::getLowerLimit(const std::string& name)
{
	return getLowerLimits()(getJointId(name));
}


Eigen::VectorXd FloatingBaseSystem::getUpperLimits()
{
	return model_.upperPositionLimit.bottomRows(n_joints_);
}


double FloatingBaseSystem::getUpperLimit(const std::string& name)
{
	return getUpperLimits()(getJointId(name));
}


Eigen::VectorXd FloatingBaseSystem::getVelocityLimits()
{
	return model_.velocityLimit.bottomRows(n_joints_);
}


double FloatingBaseSystem::getVelocityLimit(const std::string& name)
{
	return getVelocityLimits()(getJointId(name));
}


Eigen::VectorXd FloatingBaseSystem::getEffortLimits()
{
	return model_.effortLimit.bottomRows(n_joints_);
}


double FloatingBaseSystem::getEffortLimit(const std::string& name)
{
	return getEffortLimits()(getJointId(name));
}


const Eigen::VectorXd& FloatingBaseSystem::getDefaultPosture() const
{
	return default_joint_pos_;
}


bool FloatingBaseSystem::existJoint(const std::string& name) const
{
	unsigned int id = getJointId(name);
	if (id == UNDEFINED_ID)
		return false;
	else
		return true;
}


const unsigned int& FloatingBaseSystem::getBodyId(const std::string& name) const
{
	ElementId::const_iterator it = bodies_.find(name);
	if (it != bodies_.end())
		return it->second;
	else
		return UNDEFINED_ID;
}


const std::string& FloatingBaseSystem::getBodyName(const unsigned int& id) const
{
	return body_names_[id];
}


const ElementList& FloatingBaseSystem::getBodyList() const
{
	return body_names_;
}


const ElementId& FloatingBaseSystem::getBodies() const
{
	return bodies_;
}


bool FloatingBaseSystem::existBody(const std::string& name) const
{
	unsigned int id = getBodyId(name);
	if (id == UNDEFINED_ID)
		return false;
	else
		return true;
}


const unsigned int& FloatingBaseSystem::getFrameId(const std::string& name) const
{
	ElementId::const_iterator it = frames_.find(name);
	if (it != frames_.end())
		return it->second;
	else
		return UNDEFINED_ID;
}


const std::string& FloatingBaseSystem::getFrameName(const unsigned int& id) const
{
	return frame_names_[id];
}


const ElementList& FloatingBaseSystem::getFrameList() const
{
	return frame_names_;
}


const ElementId& FloatingBaseSystem::getFrames() const
{
	return frames_;
}


bool FloatingBaseSystem::existFrame(const std::string& name) const
{
	unsigned int id = getFrameId(name);
	if (id == UNDEFINED_ID)
		return false;
	else
		return true;
}


double FloatingBaseSystem::getTotalMass()
{
	double mass = 0.;
	for (int j = 1; j < model_.nbodies; ++j)
		mass += model_.inertias[j].mass();
	
	return mass;
}


double FloatingBaseSystem::getFloatingBaseMass()
{
	if (root_joint_ != NO)
		return model_.inertias[1].mass();
	else
		return model_.inertias[0].mass();
}


double FloatingBaseSystem::getBodyMass(const std::string& name)
{
	unsigned int id = getBodyId(name);
	if (id == UNDEFINED_ID)
		return std::numeric_limits<double>::quiet_NaN();
	else
		return model_.inertias[id].mass();
}


Eigen::Vector3d FloatingBaseSystem::getFloatingBaseCoM()
{
	if (root_joint_ != NO)
		return model_.inertias[1].lever();
	else
		return model_.inertias[0].lever();
}


Eigen::Vector3d FloatingBaseSystem::getBodyCoM(const std::string& name)
{
	unsigned int id = getBodyId(name);
	if (id == UNDEFINED_ID)
		return std::numeric_limits<double>::quiet_NaN() * Eigen::Vector3d::Ones();
	else
		return model_.inertias[id].lever();
}


Eigen::Matrix3d FloatingBaseSystem::getFloatingBaseInertia()
{
	if (root_joint_ != NO)
		return model_.inertias[1].inertia();
	else
		return model_.inertias[0].inertia(); // NaN values
}


Eigen::Matrix3d FloatingBaseSystem::getBodyInertia(const std::string& name)
{
	unsigned int id = getBodyId(name);
	if (id == UNDEFINED_ID)
		return std::numeric_limits<double>::quiet_NaN() * Eigen::Matrix3d::Ones();
	else
		return model_.inertias[id].inertia();
}


const unsigned int& FloatingBaseSystem::getNumberOfEndEffectors(enum TypeOfEndEffector type) const
{
	if (type == ALL)
		return num_end_effectors_;
	else
		return num_feet_;
}


const unsigned int& FloatingBaseSystem::getEndEffectorId(const std::string& contact_name) const
{
	return end_effectors_.find(contact_name)->second;
}


const ElementId& FloatingBaseSystem::getEndEffectors(enum TypeOfEndEffector type) const
{
	if (type == ALL)
		return end_effectors_;
	else
		return feet_;
}


const ElementList& FloatingBaseSystem::getEndEffectorList(enum TypeOfEndEffector type) const
{
	if (type == ALL)
		return end_effector_names_;
	else
		return foot_names_;
}


Eigen::Vector3d FloatingBaseSystem::getGravityVector()
{
	return model_.gravity.linear();
}


const double& FloatingBaseSystem::getGravityAcceleration() const
{
	return grav_acc_;
}


const Eigen::Vector3d& FloatingBaseSystem::getGravityDirection() const
{
	return grav_dir_;
}


bool FloatingBaseSystem::isFloatingBase()
{
	if (model_.joints[1].nv() == 6)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::isFixedBase()
{
	if (model_.joints[1].nv() == 1)
		return true;
	else
		return false;
}


bool FloatingBaseSystem::isConstrainedFloatingBase()
{
	if (model_.joints[1].nv() > 1 &&
		model_.joints[1].nv() != 6)
		return true;
	else
		return false;
}


const Eigen::VectorXd& FloatingBaseSystem::toConfigurationState(const dwl::SE3& base_state,
																const Eigen::VectorXd& joint_state)
{
	// Getting the number of joints
	assert(joint_state.size() == getJointDoF());

	// Note that Pinocchio defines the floating base state as
	// [linear states, angular states]
	if (root_joint_ == FREE_FLYER) {
		q_ << base_state.toVector(), joint_state;
	} else if (root_joint_ == PLANAR) {
		Eigen::VectorXd virtual_base(model_.joints[1].nq());
		Eigen::Vector7d x = base_state.toVector();
		virtual_base(1) = x(rbd::LX_Q);
		virtual_base(2) = x(rbd::LY_Q);
		virtual_base(3) = x(rbd::AZ_Q);
		virtual_base(4) = x(rbd::AW_Q);
		q_ << virtual_base, joint_state;
	} else {
		q_ = joint_state;
	}

	return q_;
}


const Eigen::VectorXd& FloatingBaseSystem::toTangentState(const dwl::Motion& base_state,
														  const Eigen::VectorXd& joint_state)
{
	// Getting the number of joints
	assert(joint_state.size() == getJointDoF());

	// Note that Pinocchio defines the floating base state as
	// [linear states, angular states]
	if (root_joint_ == FREE_FLYER) {
		v_ << base_state.toVector(), joint_state;
	} else if (root_joint_ == PLANAR) {
		Eigen::VectorXd virtual_base(model_.joints[1].nv());
		Eigen::Vector6d vr = base_state.toVector();
		virtual_base(0) = vr(rbd::LX_V);
		virtual_base(1) = vr(rbd::LY_V);
		virtual_base(2) = vr(rbd::AZ_V);
		v_ << virtual_base, joint_state;
	} else {
		v_ = joint_state;
	}

	return v_;
}


const Eigen::VectorXd& FloatingBaseSystem::toCotangentState(const dwl::Force& base_state,
															const Eigen::VectorXd& joint_state)
{
	// Getting the number of joints
	assert(joint_state.size() == getJointDoF());

	// Note that Pinocchio defines the floating base state as
	// [linear states, angular states]
	if (root_joint_ == FREE_FLYER) {
		v_ << base_state.toVector(), joint_state;
	} else if (root_joint_ == PLANAR) {
		Eigen::VectorXd virtual_base(model_.joints[1].nv());
		Eigen::Vector6d vr = base_state.toVector();
		virtual_base(0) = vr(rbd::LX_V);
		virtual_base(1) = vr(rbd::LY_V);
		virtual_base(2) = vr(rbd::AZ_V);
		v_ << virtual_base, joint_state;
	} else {
		v_ = joint_state;
	}

	return v_;
}


void FloatingBaseSystem::fromConfigurationState(dwl::SE3& base_state,
											    Eigen::VectorXd& joint_state,
											    const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint state
	joint_state.resize(getJointDoF());

	// Note that pinocchio defines the floating base state as
	// [linear states, angular states]
	if (root_joint_ == FREE_FLYER) {
		base_state.setTranslation(generalized_state.segment<3>(rbd::LX_Q));
		base_state.setQuaternion(generalized_state.segment<4>(rbd::AX_Q));
		joint_state = generalized_state.segment(7, getJointDoF());
	} else if (root_joint_ == PLANAR) {
		base_state.setTranslation(Eigen::Vector3d(generalized_state(0),
												  generalized_state(1),
												  0.));
		base_state.setQuaternion(Eigen::Vector4d(0., 0.,
												generalized_state(2),
												generalized_state(3)));
		joint_state = generalized_state.segment(4, getJointDoF());
	} else {
		base_state = dwl::SE3();
		joint_state = generalized_state;
	}
}


void FloatingBaseSystem::fromTangentState(dwl::Motion& base_state,
										  Eigen::VectorXd& joint_state,
										  const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint state
	joint_state.resize(getJointDoF());

	// Note that pinocchio defines the floating base state as
	// [linear states, angular states]
	if (root_joint_ == FREE_FLYER) {
		base_state.setLinear(generalized_state.segment<3>(rbd::LX_V));
		base_state.setAngular(generalized_state.segment<3>(rbd::AX_V));
		joint_state = generalized_state.segment(6, getJointDoF());
	} else if (root_joint_ == PLANAR) {
		base_state.setLinear(Eigen::Vector3d(generalized_state(0),
											 generalized_state(1), 0.));
		base_state.setLinear(Eigen::Vector3d(0., 0., generalized_state(2)));
		base_state.setAngular(Eigen::Vector3d::Zero());
		joint_state = generalized_state.segment(3, getJointDoF());
	} else {
		base_state.setLinear(Eigen::Vector3d::Zero());
		base_state.setAngular(Eigen::Vector3d::Zero());
		joint_state = generalized_state;
	}
}


void FloatingBaseSystem::fromCotangentState(dwl::Force& base_state,
											Eigen::VectorXd& joint_state,
											const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint state
	joint_state.resize(getJointDoF());

	// Note that pinocchio defines the floating base state as
	// [linear states, angular states]
	if (root_joint_ == FREE_FLYER) {
		base_state.setLinear(generalized_state.segment<3>(rbd::LX_V));
		base_state.setAngular(generalized_state.segment<3>(rbd::AX_V));
		joint_state = generalized_state.segment(6, getJointDoF());
	} else if (root_joint_ == PLANAR) {
		base_state.setLinear(Eigen::Vector3d(generalized_state(0),
											 generalized_state(1), 0.));
		base_state.setLinear(Eigen::Vector3d(0., 0., generalized_state(2)));
		base_state.setAngular(Eigen::Vector3d::Zero());
		joint_state = generalized_state.segment(3, getJointDoF());
	} else {
		base_state.setLinear(Eigen::Vector3d::Zero());
		base_state.setAngular(Eigen::Vector3d::Zero());
		joint_state = generalized_state;
	}
}


void FloatingBaseSystem::setBranchState(Eigen::VectorXd& new_joint_state,
										const Eigen::VectorXd& branch_state,
										std::string name)
{
	// Getting the branch properties
	unsigned int q_index, num_dof;
	getBranch(q_index, num_dof, name);

	if (branch_state.size() != num_dof) {
		printf(RED "FATAL: the branch state dimension is not consistent\n" COLOR_RESET);
		exit(EXIT_FAILURE);
	}

	new_joint_state.segment(q_index, num_dof) = branch_state;
}


Eigen::VectorXd FloatingBaseSystem::getBranchState(const Eigen::VectorXd& joint_state,
												   const std::string& name)
{
	// Getting the branch properties
	unsigned int q_index, num_dof;
	getBranch(q_index, num_dof, name);

	return joint_state.segment(q_index, num_dof);
}


bool FloatingBaseSystem::getBranch(unsigned int& pos_idx,
		   	   	   	   	   	   	   unsigned int& num_dof,
								   const std::string& name)
{
	// Getting the body id
	unsigned int frame_id = getFrameId(name);
	const se3::Frame &frame = model_.frames[frame_id];
	const se3::Model::JointIndex &joint_id = frame.parent;
	num_dof = 0;
	pos_idx = 0;

	if (frame_id == UNDEFINED_ID) {
		printf(YELLOW "Warning: undefined branch %s\n" COLOR_RESET, name.c_str());
		return false;
	}

	// Getting the base joint id. Note that the floating-base starts the
	// kinematic-tree
	unsigned int root_id = 0;
	if (root_joint_ == FREE_FLYER || root_joint_ == PLANAR)
		root_id = 1;

	// The starts of the branch is detected when its parent frame is
	// equals to the root frame
	unsigned int parent_id = joint_id;
	do {
		pos_idx = parent_id - root_id - 1;
		parent_id = model_.parents[parent_id];
		++num_dof;
	} while (parent_id != root_id);
	return true;
}


enum RootJoint FloatingBaseSystem::getRootJoint()
{
	return root_joint_;	
}


bool FloatingBaseSystem::compareString(std::string a, std::string b)
{
	return a < b;
} 

} //@namespace model
} //@namespace dwl
