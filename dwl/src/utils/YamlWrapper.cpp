#include <dwl/utils/YamlWrapper.h>


namespace dwl
{

YamlWrapper::YamlWrapper()
{

}


YamlWrapper::~YamlWrapper()
{

}


bool YamlWrapper::read(bool& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		*pnode >> data;
		return true;
	}

	return false;
}


bool YamlWrapper::read(int& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		*pnode >> data;
		return true;
	}

	return false;
}


bool YamlWrapper::read(double& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		*pnode >> data;
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::string& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		*pnode >> data;
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::vector<double>& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		int node_size = pnode->size();
		data.resize(node_size);
		for (int i = 0; i < node_size; i++) {
			double number;

			if (const YAML::Node* inode = pnode->FindValue(i)) {
				*inode >> number;
				data[i] = number;
			}
		}

		return true;
	}

	return false;
}


bool YamlWrapper::read(std::vector<std::string>& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		int node_size = pnode->size();
		data.resize(node_size);
		for (int i = 0; i < node_size; i++) {
			std::string str;

			if (const YAML::Node* inode = pnode->FindValue(i)) {
				*inode >> str;
				data[i] = str;
			}
		}
		return true;
	}

	return false;
}


bool YamlWrapper::read(Eigen::Vector2d& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		for (int i = 0; i < 2; i++) {
			if (const YAML::Node* inode = pnode->FindValue(i)) {
				*inode >> data(i);
			}
		}
		return true;
	}

	return false;
}


bool YamlWrapper::read(Eigen::Vector3d& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		for (int i = 0; i < 3; i++) {
			if (const YAML::Node* inode = pnode->FindValue(i)) {
				*inode >> data(i);
			}
		}
		return true;
	}

	return false;
}


bool YamlWrapper::read(Eigen::Quaterniond& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		Eigen::VectorXd q = Eigen::VectorXd::Zero(4); // (w,x,y,z)^T
		for (int i = 0; i < 4; i++) {
			if (const YAML::Node* inode = pnode->FindValue(i)) {
				*inode >> q(i);
			}
		}

		Eigen::Quaterniond orientation(q(0), q(1), q(2), q(3));
		data = orientation;
		return true;
	}

	return false;
}


bool YamlWrapper::read(Pose& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		// Reading the position data
		if (!read(data.position, *pnode, "position"))
			data.position = Eigen::Vector3d::Zero();

		// Reading the orientation data
		if (!read(data.orientation, *pnode, "orientation")) {
			Eigen::Quaterniond default_orientation(1, 0, 0, 0);
			data.orientation = default_orientation;
		}

		return true;
	}

	return false;
}


bool YamlWrapper::read(Pose3d& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		// Reading the position data
		if (!read(data.position, *pnode, "position"))
			data.position = Eigen::Vector2d::Zero();

		// Reading the orientation data
		if (!read(data.orientation, *pnode, "orientation"))
			data.orientation = 0.;

		return true;
	}

	return false;
}


bool YamlWrapper::read(Action3d& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		// Reading pose data
		if (!read(data.pose, *pnode, "pose")) {
			data.pose.position = Eigen::Vector2d::Zero();
			data.pose.orientation = 0;
		}

		// Reading the cost data
		if (!read(data.cost, *pnode, "cost"))
			data.cost = 0;

		return true;
	}

	return false;
}


bool YamlWrapper::read(SearchArea& data,
					   const YAML::Node& node,
					   std::string field_name)
{
	if (const YAML::Node* pnode = node.FindValue(field_name)) {
		// Reading the vertices
		if (!read(data.min_x, *pnode, "min_x"))
			data.min_x = 0;
		if (!read(data.max_x, *pnode, "max_x"))
			data.max_x = 0;

		if (!read(data.min_y, *pnode, "min_y"))
			data.min_y = 0;
		if (!read(data.max_y, *pnode, "max_y"))
			data.max_y = 0;

		if (!read(data.min_z, *pnode, "min_z"))
			data.min_z = 0;
		if (!read(data.max_z, *pnode, "max_z"))
			data.max_z = 0;

		// Reading the resolution
		if (!read(data.resolution, *pnode, "resolution"))
			data.resolution = 0;

		return true;
	}

	return false;
}

}
