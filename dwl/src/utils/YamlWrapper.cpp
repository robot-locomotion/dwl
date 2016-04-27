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
					   std::string field)
{
	if (node[field]) {
		data = node[field].as<bool>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(int& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		data = node[field].as<int>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(double& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		data = node[field].as<double>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::string& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		data = node[field].as<std::string>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::vector<double>& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		data = node[field].as<std::vector<double>>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(std::vector<std::string>& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		data = node[field].as<std::vector<std::string>>();
		return true;
	}

	return false;
}


bool YamlWrapper::read(Eigen::Vector2d& data,
					   const YAML::Node& node,
					   std::string field)
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
					   const YAML::Node& node,
					   std::string field)
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
					   const YAML::Node& node,
					   std::string field)
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
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		// Reading the position data
		if (!read(data.position, node[field], "position"))
			data.position = Eigen::Vector3d::Zero();

		// Reading the orientation data
		if (!read(data.orientation, node[field], "orientation")) {
			Eigen::Quaterniond default_orientation(1, 0, 0, 0);
			data.orientation = default_orientation;
		}

		return true;
	}

	return false;
}


bool YamlWrapper::read(Pose3d& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		// Reading the position data
		if (!read(data.position, node[field], "position"))
			data.position = Eigen::Vector2d::Zero();

		// Reading the orientation data
		if (!read(data.orientation, node[field], "orientation")) {
			data.orientation = 0.;
		}

		return true;
	}

	return false;
}


bool YamlWrapper::read(Action3d& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		// Reading pose data
		if (!read(data.pose, node[field], "pose")) {
			data.pose.position = Eigen::Vector2d::Zero();
			data.pose.orientation = 0.;
		}

		// Reading the cost data
		if (!read(data.cost, node[field], "cost"))
			data.cost = 0.;

		return true;
	}

	return false;
}


bool YamlWrapper::read(SearchArea& data,
					   const YAML::Node& node,
					   std::string field)
{
	if (node[field]) {
		// Reading the vertices
		if (!read(data.min_x, node[field], "min_x"))
			data.min_x = 0.;
		if (!read(data.max_x, node[field], "max_x"))
			data.max_x = 0.;

		if (!read(data.min_y, node[field], "min_y"))
			data.min_y = 0.;
		if (!read(data.max_y, node[field], "max_y"))
			data.max_y = 0.;

		if (!read(data.min_z, node[field], "min_z"))
			data.min_z = 0.;
		if (!read(data.max_z, node[field], "max_z"))
			data.max_z = 0.;

		if (!read(data.resolution, node[field], "resolution"))
			data.resolution = 0.;

		return true;
	}

	return false;
}

}
