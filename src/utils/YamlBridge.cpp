#include <utils/YamlBridge.h>


namespace dwl
{

YamlBridge::YamlBridge()
{

}


YamlBridge::~YamlBridge()
{

}


void YamlBridge::read(const YAML::Node& node, double& data)
{
	node >> data;
}


void YamlBridge::read(const YAML::Node& node, std::string& data)
{
	node >> data;
}


void YamlBridge::read(const YAML::Node& node, std::vector<double>& data)
{
	int node_size = node.size();
	for (unsigned int i = 0; i < node_size; i++) {
		double number;

		read(node[i], number);
		data.push_back(number);
	}
}


void YamlBridge::read(const YAML::Node& node, std::vector<std::string>& data)
{
	int node_size = node.size();
	for (unsigned int i = 0; i < node_size; i++) {
		std::string str;

		read(node[i], str);
		data.push_back(str);
	}
}


void YamlBridge::read(const YAML::Node& node, Eigen::Vector2d& data)
{
	node[0] >> data(0);
	node[1] >> data(1);
}


void YamlBridge::read(const YAML::Node& node, Eigen::Vector3d& data)
{
	node[0] >> data(0);
	node[1] >> data(1);
	node[2] >> data(2);
}


void YamlBridge::read(const YAML::Node& node, Eigen::Quaterniond& data)
{
	double w, x, y, z;
	read(node[0], w);
	read(node[1], x);
	read(node[2], y);
	read(node[3], z);

	Eigen::Quaterniond orientation(w, x, y, z);
	data = orientation;
}


void YamlBridge::read(const YAML::Node& node, Pose& data)
{
	if (const YAML::Node* pposition = node.FindValue("position"))
		read(*pposition, data.position);
	else
		data.position = Eigen::Vector3d::Zero();

	if (const YAML::Node* porientation = node.FindValue("orientation"))
		read(*porientation, data.orientation);
	else {
		Eigen::Quaterniond default_orientation(1, 0, 0, 0);
		data.orientation = default_orientation;
	}
}


void YamlBridge::read(const YAML::Node& node, Pose3d& data)
{
	if (const YAML::Node* pposition = node.FindValue("position"))
		read(*pposition, data.position);
	else
		data.position = Eigen::Vector2d::Zero();

	if (const YAML::Node* porientation = node.FindValue("orientation"))
		read(*porientation, data.orientation);
	else
		data.orientation = 0;
}


void YamlBridge::read(const YAML::Node& node, Action3d& data)
{
	if (const YAML::Node* ppose = node.FindValue("pose"))
		read(*ppose, data.pose);
	else {
		data.pose.position = Eigen::Vector2d::Zero();
		data.pose.orientation = 0;
	}

	if (const YAML::Node* pcost = node.FindValue("cost"))
		read(*pcost, data.cost);
	else
		data.cost = 0;
}


void YamlBridge::read(const YAML::Node& node, SearchArea& data)
{
	if (const YAML::Node* pmin_x = node.FindValue("min_x"))
		read(*pmin_x, data.min_x);
	else
		data.min_x = 0;

	if (const YAML::Node* pmax_x = node.FindValue("max_x"))
		read(*pmax_x, data.max_x);
	else
		data.max_x = 0;

	if (const YAML::Node* pmin_y = node.FindValue("min_y"))
		read(*pmin_y, data.min_y);
	else
		data.min_y = 0;

	if (const YAML::Node* pmax_y = node.FindValue("max_y"))
		read(*pmax_y, data.max_y);
	else
		data.max_y = 0;

	if (const YAML::Node* pmin_z = node.FindValue("min_z"))
		read(*pmin_z, data.min_z);
	else
		data.min_z = 0;

	if (const YAML::Node* pmax_z = node.FindValue("max_z"))
		read(*pmax_z, data.max_z);
	else
		data.max_z = 0;

	if (const YAML::Node* presolution = node.FindValue("resolution"))
		read(*presolution, data.resolution);
	else
		data.resolution = 0;
}

}
