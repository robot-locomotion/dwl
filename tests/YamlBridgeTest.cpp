#include <utils/YamlBridge.h>


int main(int argc, char **argv)
{
	// variables
	std::vector<double> double_vec;
	std::vector<std::string> string_vec;
	Eigen::Vector3d vector_3d;
	Eigen::Quaterniond quaternion;
	dwl::Pose pose;
	dwl::Pose3d pose3d;
	dwl::Action3d action3d;
	dwl::SearchArea search_area;

	// Yaml reader
	dwl::YamlBridge yaml_reader;

	// Reading and parsing the yaml document
	std::ifstream fin("../tests/test.yaml");
	YAML::Parser parser(fin);
	YAML::Node doc;

	parser.GetNextDocument(doc);
	for (YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
		// Reading the name of the robot
		std::string namespace_file;
		it.first() >> namespace_file;
		std::cout << "Reading from " << namespace_file << std::endl;

		// Reading the test variables
		if (const YAML::Node* ptest = doc.FindValue(namespace_file)) {
			const YAML::Node& test = *ptest;

			// Reading the double vector
			if (const YAML::Node* pdouble_vec = test.FindValue("double_vector")) {
				yaml_reader.read(*pdouble_vec, double_vec);
			}

			// Reading the string vector
			if (const YAML::Node* pstring_vec = test.FindValue("string_vector")) {
				yaml_reader.read(*pstring_vec, string_vec);
			}

			// Reading the 3d vector
			if (const YAML::Node* pvector_3d = test.FindValue("vector_3d")) {
				yaml_reader.read(*pvector_3d, vector_3d);
			}

			// Reading the quaternion
			if (const YAML::Node* pquaternion = test.FindValue("quaternion")) {
				yaml_reader.read(*pquaternion, quaternion);
			}

			// Reading the pose
			if (const YAML::Node* ppose = test.FindValue("pose")) {
				yaml_reader.read(*ppose, pose);
			}

			// Reading the pose 3d
			if (const YAML::Node* ppose3d = test.FindValue("pose3d")) {
				yaml_reader.read(*ppose3d, pose3d);
			}

			// Reading the action 3d
			if (const YAML::Node* paction3d = test.FindValue("action3d")) {
				yaml_reader.read(*paction3d, action3d);
			}

			// Reading the search area
			if (const YAML::Node* psearch_area = test.FindValue("search_area")) {
				yaml_reader.read(*psearch_area, search_area);
			}
		}
	}

	// Print the read variables
	for (unsigned int i = 0; i < double_vec.size(); i++) {
		if (i == 0)
			std::cout << "Double values = " << double_vec[i] << " ";
		else if (i == double_vec.size() - 1)
			std::cout << double_vec[i] << std::endl;
		else
			std::cout << double_vec[i] << " ";
	}

	for (unsigned int i = 0; i < string_vec.size(); i++) {
		if (i == 0)
			std::cout << "String values = " << string_vec[i] << " ";
		else if (i == string_vec.size() - 1)
			std::cout << string_vec[i] << std::endl;
		else
			std::cout << string_vec[i] << " ";
	}

	for (unsigned int i = 0; i < 3; i++) {
		if (i == 0)
			std::cout << "Vector 3d values = " << vector_3d(i) << " ";
		else if (i == 2)
			std::cout << (double) vector_3d(i) << std::endl;
		else
			std::cout << (double) vector_3d(i) << " ";
	}

	std::cout << "Quaternion values = " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << std::endl;

	std::cout << "Pose.position values = " << pose.position(0) << " " << pose.position(1) << " " << pose.position(2) << std::endl;
	std::cout << "Pose.orientation values = " << pose.orientation.w() << " " << pose.orientation.x() << " " << pose.orientation.y() << " " << pose.orientation.z() << std::endl;

	std::cout << "Pose3d.position values = " << pose3d.position(0) << " " << pose3d.position(1) << " " << pose3d.position(2) << std::endl;
	std::cout << "Pose3d.orientation value = " << pose3d.orientation << std::endl;

	std::cout << "Action3d.pose values = " << action3d.pose.position(0) << " " << action3d.pose.position(1) << " " << action3d.pose.orientation << std::endl;
	std::cout << "Action3d.cost value = " << action3d.cost << std::endl;

	std::cout << "Search area min = " << search_area.min_x << " " << search_area.min_y << " " << search_area.min_z << std::endl;
	std::cout << "Search area max = " << search_area.max_x << " " << search_area.max_y << " " << search_area.max_z << std::endl;
	std::cout << "Search area resolution = " << search_area.resolution << std::endl;
}
