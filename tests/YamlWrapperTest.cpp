#include <dwl/utils/YamlWrapper.h>
#include "yaml-cpp/yaml.h"


using namespace std;

int main(int argc, char **argv)
{
	// variables
	int idata;
	double ddata;
	string sdata;
	bool bdata;
	vector<double> double_vec;
	vector<string> string_vec;
	Eigen::Vector2d vector_2d;
	Eigen::Vector3d vector_3d;
	Eigen::Quaterniond quaternion;
	dwl::Pose pose;
	dwl::Pose3d pose_3d;
	dwl::Action3d action_3d;
	dwl::SearchArea search_area;

	// Yaml reader
	dwl::YamlWrapper yaml_reader("../tests/test.yaml");

	// Reading and parsing the yaml document
	cout << "Reading from global_ns/variable_ns namespace" << endl;


	// Reading the test variables
	dwl::YamlNamespace ns = {"global_ns", "variable_ns"};
	if (yaml_reader.read(idata, "int",  ns))
		cout << "int: " << idata << endl;
	else
		cout << "Couldn't read int tag" << endl;

	if (yaml_reader.read(ddata, "double", ns))
		cout << "double: " << ddata << endl;
	else
		cout << "Couldn't read double tag" << endl;

	if (yaml_reader.read(sdata, "string", ns))
		cout << "string: " << sdata << endl;
	else
		cout << "Couldn't read string tag" << endl;

	if (yaml_reader.read(bdata, "bool", ns))
		cout << "bool: " << bdata << endl;
	else
		cout << "Couldn't read bool tag" << endl;

	if (yaml_reader.read(double_vec, "double_vector", ns)) {
		cout << "double_vector: ";
		for (size_t i = 0; i < double_vec.size(); i++)
			cout << double_vec[i] << " ";
		cout << endl;
	} else
		cout << "Couldn't read double_vector tag" << endl;

	if (yaml_reader.read(string_vec, "string_vector", ns)) {
		cout << "string_vector: ";
		for (size_t i = 0; i < string_vec.size(); i++)
			cout << string_vec[i] << " ";
		cout << endl;
	} else
		cout << "Couldn't read string_vector tag" << endl;

	if (yaml_reader.read(vector_2d, "vector_2d", ns)) {
		cout << "vector_2d: " << vector_2d.transpose() << endl;
	} else
		cout << "Couldn't read vector_2d tag" << endl;

	if (yaml_reader.read(vector_3d, "vector_3d", ns)) {
		cout << "vector_3d: " << vector_3d.transpose() << endl;
	} else
		cout << "Couldn't read vector_3d tag" << endl;

	if (yaml_reader.read(quaternion, "quaternion", ns)) {
		cout << "quaternion = " << quaternion.w() << " "
								<< quaternion.x() << " "
								<< quaternion.y() << " "
								<< quaternion.z() << endl;
	} else
		cout << "Couldn't read quaternion tag" << endl;

	if (yaml_reader.read(pose, "pose", ns)) {
		cout << "pose.position = " << pose.position.transpose() << endl;
		cout << "pose.orientation = " << pose.orientation.w() << " "
									  << pose.orientation.x() << " "
									  << pose.orientation.y() << " "
									  << pose.orientation.z() << endl;
	} else
		cout << "Couldn't read pose tag" << endl;

	if (yaml_reader.read(pose_3d, "pose_3d", ns)) {
		cout << "pose_3d.position = " << pose_3d.position.transpose() << endl;
		cout << "pose_3d.orientation = " << pose_3d.orientation << endl;
	} else
		cout << "Couldn't read pose_3d tag" << endl;

	if (yaml_reader.read(action_3d, "action_3d", ns)) {
		cout << "action_3d.pose.position = " << action_3d.pose.position.transpose() << endl;
		cout << "action_3d.pose.orientation = " << action_3d.pose.orientation << endl;
		cout << "action_3d.cost = " << action_3d.cost << endl;
	} else
		cout << "Couldn't read action_3d tag" << endl;

	if (yaml_reader.read(search_area, "search_area", ns)) {
		cout << "search_area.min_x = " << search_area.min_x << endl;
		cout << "search_area.max_x = " << search_area.max_x << endl;
		cout << "search_area.min_y = " << search_area.min_y << endl;
		cout << "search_area.max_y = " << search_area.max_y << endl;
		cout << "search_area.min_z = " << search_area.min_z << endl;
		cout << "search_area.max_z = " << search_area.max_z << endl;
		cout << "search_area.resolution = " << search_area.resolution << endl;
	} else
		cout << "Couldn't read search_area tag" << endl;




	YAML::Emitter out;
//	out << file;
	out << YAML::BeginMap;
	out << YAML::Key << "my_ns";
	out << YAML::BeginMap;
	out << YAML::Key << "my_2ns";
	out << YAML::Value;
	out << YAML::Value;
	out << YAML::BeginSeq;
	out << "eggs";
	out << "bread";
	out << "milk";
	out << YAML::EndSeq;
	out << YAML::EndMap;
	out << YAML::EndMap;

	std::cout << "Here's the output YAML:\n" << out.c_str() << std::endl; // prints "Hello, World!"
//	std::ofstream fout;
//	fout.open("tmp.yaml", fstream::out | fstream::app);
//	fout << out.c_str();
//	fout.close();

	return 0;
}
