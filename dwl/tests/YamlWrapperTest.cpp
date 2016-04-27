#include <dwl/utils/YamlWrapper.h>

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
	dwl::YamlWrapper yaml_reader;

	// Reading and parsing the yaml document
	YAML::Node file = YAML::LoadFile("../tests/test.yaml");
	cout << "Reading from test namespace" << endl;// << file


	// Reading the test variables
	if (yaml_reader.read(idata, file["test"], "int"))
		cout << "int: " << idata << endl;

	if (yaml_reader.read(ddata, file["test"], "double"))
		cout << "double: " << ddata << endl;

	if (yaml_reader.read(sdata, file["test"], "string"))
		cout << "string: " << sdata << endl;

	if (yaml_reader.read(bdata, file["test"], "bool"))
		cout << "bool: " << bdata << endl;

	if (yaml_reader.read(double_vec, file["test"], "double_vector")) {
		cout << "double_vector: ";
		for (size_t i = 0; i < double_vec.size(); i++)
			cout << double_vec[i] << " ";
		cout << endl;
	}

	if (yaml_reader.read(string_vec, file["test"], "string_vector")) {
		cout << "string_vector: ";
		for (size_t i = 0; i < string_vec.size(); i++)
			cout << string_vec[i] << " ";
		cout << endl;
	}

	if (yaml_reader.read(vector_2d, file["test"], "vector_2d")) {
		cout << "vector_2d: " << vector_2d.transpose() << endl;
	}

	if (yaml_reader.read(vector_3d, file["test"], "vector_3d")) {
		cout << "vector_3d: " << vector_3d.transpose() << endl;
	}

	if (yaml_reader.read(quaternion, file["test"], "quaternion")) {
		cout << "quaternion = " << quaternion.w() << " "
								<< quaternion.x() << " "
								<< quaternion.y() << " "
								<< quaternion.z() << endl;
	}

	if (yaml_reader.read(pose, file["test"], "pose")) {
		cout << "pose.position = " << pose.position.transpose() << endl;
		cout << "pose.orientation = " << pose.orientation.w() << " "
									  << pose.orientation.x() << " "
									  << pose.orientation.y() << " "
									  << pose.orientation.z() << endl;
	}

	if (yaml_reader.read(pose_3d, file["test"], "pose_3d")) {
		cout << "pose_3d.position = " << pose_3d.position.transpose() << endl;
		cout << "pose_3d.orientation = " << pose_3d.orientation << endl;
	}

	if (yaml_reader.read(action_3d, file["test"], "action_3d")) {
		cout << "action_3d.pose.position = " << action_3d.pose.position.transpose() << endl;
		cout << "action_3d.pose.orientation = " << action_3d.pose.orientation << endl;
		cout << "action_3d.cost = " << action_3d.cost << endl;
	}

	if (yaml_reader.read(search_area, file["test"], "search_area")) {
		cout << "search_area.min_x = " << search_area.min_x << endl;
		cout << "search_area.max_x = " << search_area.max_x << endl;
		cout << "search_area.min_y = " << search_area.min_y << endl;
		cout << "search_area.max_y = " << search_area.max_y << endl;
		cout << "search_area.min_z = " << search_area.min_z << endl;
		cout << "search_area.max_z = " << search_area.max_z << endl;
		cout << "search_area.resolution = " << search_area.resolution << endl;
	}
}
