#include <utils/URDF.h>


namespace dwl
{

namespace urdf_model
{

void getJointNames(JointID& joints,
				   std::string urdf_model,
				   enum JointType type)
{
	// Parsing the URDF-XML
	boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF(urdf_model);

	std::stack<boost::shared_ptr<urdf::Link> > link_stack;
	std::stack<int> branch_index_stack;

	// Adding the bodies in a depth-first order of the model tree
	std::map<std::string, boost::shared_ptr<urdf::Link> > link_map = model->links_;
	link_stack.push(link_map[(model->getRoot()->name)]);

	if (link_stack.top()->child_joints.size() > 0) {
		branch_index_stack.push(0);
	}

	unsigned joint_idx = 0;
	while (link_stack.size() > 0) {
		boost::shared_ptr<urdf::Link> current_link = link_stack.top();
		unsigned int branch_idx = branch_index_stack.top();

		if (branch_idx < current_link->child_joints.size()) {
			boost::shared_ptr<urdf::Joint> current_joint = current_link->child_joints[branch_idx];

			// Incrementing branch index
			branch_index_stack.pop();
			branch_index_stack.push(branch_idx + 1);

			link_stack.push(link_map[current_joint->child_link_name]);
			branch_index_stack.push(0);

			// Searching joints names
			if (type == free) {
				if (current_joint->type == urdf::Joint::FLOATING ||
						current_joint->type == urdf::Joint::PRISMATIC ||
						current_joint->type == urdf::Joint::REVOLUTE ||
						current_joint->type == urdf::Joint::CONTINUOUS) {
					joints[current_joint->name] = joint_idx;
					joint_idx++;
				}
			} else if (type == fixed) {
				if (current_joint->type == urdf::Joint::FIXED) {
					joints[current_joint->name] = joint_idx;
					joint_idx++;
				}
			} else { // both joints
				if (current_joint->type == urdf::Joint::FIXED)
					joints[current_joint->name] = std::numeric_limits<unsigned int>::max();
				else {
					joints[current_joint->name] = joint_idx;
					joint_idx++;
				}
			}
		} else {
			link_stack.pop();
			branch_index_stack.pop();
		}
	}
}


void getEndEffectors(LinkSelector& end_effectors,
					 std::string urdf_model)
{
	// Parsing the URDF-XML
	boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF(urdf_model);

	// Getting the joint names
	JointID fixed_joints;
	getJointNames(fixed_joints, urdf_model, fixed);

	// Getting the world, root, parent and child links
	boost::shared_ptr<urdf::Link> world_link = model->links_[model->getRoot()->name];
	boost::shared_ptr<urdf::Link> root_link = world_link->child_links[0];

	// Searching the end-effector joints
	for (urdf_model::JointID::iterator jnt_it = fixed_joints.begin();
			jnt_it != fixed_joints.end(); jnt_it++) {
		std::string joint_name = jnt_it->first;
		boost::shared_ptr<urdf::Joint> current_joint = model->joints_[joint_name];

		// Getting the parent and child link of the current fixed joint
		boost::shared_ptr<urdf::Link> parent_link = model->links_[current_joint->parent_link_name];
		boost::shared_ptr<urdf::Link> child_link = model->links_[current_joint->child_link_name];

		// Checking if it's an end-effector
		unsigned int num_childs = child_link->child_joints.size();
		while (parent_link->name != root_link->name && num_childs == 0) {
			boost::shared_ptr<urdf::Joint> parent_joint = model->joints_[parent_link->parent_joint->name];
			if (parent_joint->type == urdf::Joint::PRISMATIC ||
					parent_joint->type == urdf::Joint::REVOLUTE) {
				end_effectors.push_back(current_joint->child_link_name);
				break;
			}

			parent_link = parent_link->getParent();
		}
	}
}


void getJointLimits(Eigen::VectorXd& lower_joint_pos,
					Eigen::VectorXd& upper_joint_pos,
					Eigen::VectorXd& joint_vel,
					Eigen::VectorXd& joint_eff,
					std::string urdf_model)
{
	// Parsing the URDF-XML
	boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF(urdf_model);

	// Getting the joint names
	JointID joints;
	getJointNames(joints, urdf_model, free);

	// Computing the number of actuated joints
	unsigned num_joints = 0;
	for (urdf_model::JointID::iterator jnt_it = joints.begin();
			jnt_it != joints.end(); jnt_it++) {
		std::string joint_name = jnt_it->first;
		boost::shared_ptr<urdf::Joint> current_joint = model->joints_[joint_name];

		if (current_joint->type != urdf::Joint::FLOATING)
			if (current_joint->limits->effort != 0) // Virtual floating-base joints
				num_joints++;
	}

	// Resizing the vectors
	lower_joint_pos.resize(num_joints);
	upper_joint_pos.resize(num_joints);
	joint_vel.resize(num_joints);
	joint_eff.resize(num_joints);

	// Searching the joint limits
	unsigned int joint_idx = 0;
	for (urdf_model::JointID::iterator jnt_it = joints.begin();
			jnt_it != joints.end(); jnt_it++) {
		std::string joint_name = jnt_it->first;
		boost::shared_ptr<urdf::Joint> current_joint = model->joints_[joint_name];

		if (current_joint->type != urdf::Joint::FLOATING) {
			if (current_joint->limits->effort != 0) {  // Virtual floating-base joints
				lower_joint_pos(joint_idx) = current_joint->limits->lower;
				upper_joint_pos(joint_idx) = current_joint->limits->upper;
				joint_vel(joint_idx) = current_joint->limits->velocity;
				joint_eff(joint_idx) = current_joint->limits->effort;
				joint_idx++;
			}
		}
	}
}

} //@namespace urdf_model
} //@namespace dwl
