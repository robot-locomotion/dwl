#include <dwl/utils/YamlWrapper.h>


namespace dwl
{

YamlWrapper::YamlWrapper() : is_file_(false)
{

}


YamlWrapper::YamlWrapper(std::string filename) : filename_(filename),
		is_file_(true)
{

}


YamlWrapper::~YamlWrapper()
{

}


void YamlWrapper::setFile(std::string filename)
{
	filename_ = filename;
	is_file_ = true;
}


bool YamlWrapper::read(bool& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(bool& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		data = node[field].as<bool>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(int& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(int& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		data = node[field].as<int>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(double& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(double& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		data = node[field].as<double>();
		return true;
	}

	return false;
}


//	return false;
//}
bool YamlWrapper::read(std::string& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(std::string& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		data = node[field].as<std::string>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::vector<double>& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(std::vector<double>& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		data = node[field].as<std::vector<double>>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::vector<std::string>& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(std::vector<std::string>& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		data = node[field].as<std::vector<std::string>>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(Eigen::Vector2d& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(Eigen::Vector2d& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		if (node[field].size() == 2) {
			for (std::size_t i = 0; i < 2; i++)
				data(i) = node[field][i].as<double>();
			return true;
		}
	}

	return false;
}


bool YamlWrapper::read(Eigen::Vector3d& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(Eigen::Vector3d& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		if (node[field].size() == 3) {
			for (std::size_t i = 0; i < 3; i++)
				data(i) = node[field][i].as<double>();
			return true;
		}
	}

	return false;
}


bool YamlWrapper::read(Eigen::Quaterniond& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(Eigen::Quaterniond& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	if (node[field]) {
		if (node[field].size() == 4) {
			Eigen::VectorXd q = Eigen::VectorXd::Zero(4); // (w,x,y,z)^T
			for (std::size_t i = 0; i < 4; i++)
				q(i) = node[field][i].as<double>();

			Eigen::Quaterniond orientation(q(0), q(1), q(2), q(3));
			data = orientation;
			return true;
		}
	}

	return false;
}


bool YamlWrapper::read(Pose& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(Pose& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	YAML::Node local_node = node[field];
	if (local_node) {
		// Reading the position data
		if (!read(data.position, "position", local_node))
			data.position = Eigen::Vector3d::Zero();

		// Reading the orientation data
		if (!read(data.orientation, "orientation", local_node)) {
			Eigen::Quaterniond default_orientation(1, 0, 0, 0);
			data.orientation = default_orientation;
		}

		return true;
	}

	return false;
}


bool YamlWrapper::read(Pose3d& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(Pose3d& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	YAML::Node local_node = node[field];
	if (local_node) {
		// Reading the position data
		if (!read(data.position, "position", local_node))
			data.position = Eigen::Vector2d::Zero();

		// Reading the orientation data
		if (!read(data.orientation, "orientation", local_node)) {
			data.orientation = 0.;
		}

		return true;
	}

	return false;
}


bool YamlWrapper::read(Action3d& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(Action3d& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	YAML::Node local_node = node[field];
	if (local_node) {
		// Reading pose data
		if (!read(data.pose, "pose", local_node)) {
			data.pose.position = Eigen::Vector2d::Zero();
			data.pose.orientation = 0.;
		}

		// Reading the cost data
		if (!read(data.cost, "cost", local_node))
			data.cost = 0.;

		return true;
	}

	return false;
}


bool YamlWrapper::read(SearchArea& data,
					   const std::string& field,
					   const YamlNamespace& ns)
{
	// Finding the node of the respective namespaces
	YAML::Node node;
	if (!getNode(node, ns))
		return false;

	return read(data, field, node);
}


bool YamlWrapper::read(SearchArea& data,
					   const std::string& field,
					   const YAML::Node& node)
{
	YAML::Node local_node = node[field];
	if (local_node) {
		// Reading the vertices
		if (!read(data.min_x, "min_x", local_node))
			data.min_x = 0.;
		if (!read(data.max_x, "max_x", local_node))
			data.max_x = 0.;

		if (!read(data.min_y, "min_y", local_node))
			data.min_y = 0.;
		if (!read(data.max_y, "max_y", local_node))
			data.max_y = 0.;

		if (!read(data.min_z, "min_z", local_node))
			data.min_z = 0.;
		if (!read(data.max_z, "max_z", local_node))
			data.max_z = 0.;

		if (!read(data.resolution, "resolution", local_node))
			data.resolution = 0.;

		return true;
	}

	return false;
}


bool YamlWrapper::getNode(YAML::Node& node,
						  const YamlNamespace& ns)
{
	if (!is_file_) {
		printf(YELLOW "Warning: the filename needs to be defined\n" COLOR_RESET);
		return false;
	}

	node = YAML::LoadFile(filename_);
	for (std::size_t i = 0; i < ns.size(); i++) {
		std::string ns_name = ns[i];

		node = node[ns_name];
		if (!node)
			return false;
	}

	return true;
}

}
