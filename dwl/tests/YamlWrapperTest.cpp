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
	dwl::YamlWrapper yaml_reader("../tests/test.yaml");

	// Reading and parsing the yaml document
	cout << "Reading from test namespace" << endl;

	// Reading the test variables
	if (yaml_reader.read(idata, "int", {"global_ns", "variable_ns"}))
		cout << "int: " << idata << endl;

	if (yaml_reader.read(ddata, "double", {"global_ns", "variable_ns"}))
		cout << "double: " << ddata << endl;

	if (yaml_reader.read(sdata, "string", {"global_ns", "variable_ns"}))
		cout << "string: " << sdata << endl;


	if (yaml_reader.read(bdata, "bool", {"global_ns", "variable_ns"}))
		cout << "bool: " << bdata << endl;

	if (yaml_reader.read(double_vec, "double_vector", {"global_ns", "variable_ns"})) {
		cout << "double_vector: ";
		for (size_t i = 0; i < double_vec.size(); i++)
			cout << double_vec[i] << " ";
		cout << endl;
	}

	if (yaml_reader.read(string_vec, "string_vector", {"global_ns", "variable_ns"})) {
		cout << "string_vector: ";
		for (size_t i = 0; i < string_vec.size(); i++)
			cout << string_vec[i] << " ";
		cout << endl;
	}

	if (yaml_reader.read(vector_2d, "vector_2d", {"global_ns", "variable_ns"})) {
		cout << "vector_2d: " << vector_2d.transpose() << endl;
	}

	if (yaml_reader.read(vector_3d, "vector_3d", {"global_ns", "variable_ns"})) {
		cout << "vector_3d: " << vector_3d.transpose() << endl;
	}

	if (yaml_reader.read(quaternion, "quaternion", {"global_ns", "variable_ns"})) {
		cout << "quaternion = " << quaternion.w() << " "
								<< quaternion.x() << " "
								<< quaternion.y() << " "
								<< quaternion.z() << endl;
	}

	if (yaml_reader.read(pose, "pose", {"global_ns", "variable_ns"})) {
		cout << "pose.position = " << pose.position.transpose() << endl;
		cout << "pose.orientation = " << pose.orientation.w() << " "
									  << pose.orientation.x() << " "
									  << pose.orientation.y() << " "
									  << pose.orientation.z() << endl;
	}

	if (yaml_reader.read(pose_3d, "pose_3d", {"global_ns", "variable_ns"})) {
		cout << "pose_3d.position = " << pose_3d.position.transpose() << endl;
		cout << "pose_3d.orientation = " << pose_3d.orientation << endl;
	}

	if (yaml_reader.read(action_3d, "action_3d", {"global_ns", "variable_ns"})) {
		cout << "action_3d.pose.position = " << action_3d.pose.position.transpose() << endl;
		cout << "action_3d.pose.orientation = " << action_3d.pose.orientation << endl;
		cout << "action_3d.cost = " << action_3d.cost << endl;
	}

	if (yaml_reader.read(search_area, "search_area", {"global_ns", "variable_ns"})) {
		cout << "search_area.min_x = " << search_area.min_x << endl;
		cout << "search_area.max_x = " << search_area.max_x << endl;
		cout << "search_area.min_y = " << search_area.min_y << endl;
		cout << "search_area.max_y = " << search_area.max_y << endl;
		cout << "search_area.min_z = " << search_area.min_z << endl;
		cout << "search_area.max_z = " << search_area.max_z << endl;
		cout << "search_area.resolution = " << search_area.resolution << endl;
	}
	return 0;
}
