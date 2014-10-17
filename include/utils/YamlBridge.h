#include <utils/utils.h>
#include <fstream>
#include <yaml-cpp/yaml.h>


namespace dwl
{

/**
 * @class YamlBridge
 * @brief Class for reading different yaml structures of the DWL
 */
class YamlBridge
{
	public:
		/** @brief Constructor function */
		YamlBridge();

		/** @brief Destructor function */
		~YamlBridge();

		/**
		 * @brief Reads a double
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param double& data Data to read
		 */

		void read(const YAML::Node& node, double& data);

		/**
		 * @brief Reads a string
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param std::string& data Data to read
		 */
		void read(const YAML::Node& node, std::string& data);

		/**
		 * @brief Reads a std vector of doubles
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param std::vector<double>& data Data to read
		 */
		void read(const YAML::Node& node, std::vector<double>& data);

		/**
		 * @brief Reads a std vector of string
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param std::vector<string>& data Data to read
		 */
		void read(const YAML::Node& node, std::vector<std::string>& data);

		/**
		 * @brief Reads a 2d vector
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param Eigen::Vector2d& data Data to read
		 */
		void read(const YAML::Node& node, Eigen::Vector2d& data);

		/**
		 * @brief Reads a 3d vector
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param Eigen::Vector3d& data Data to read
		 */
		void read(const YAML::Node& node, Eigen::Vector3d& data);

		/**
		 * @brief Reads a quaternion
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param Eigen::Quaterniond& data Data to read
		 */
		void read(const YAML::Node& node, Eigen::Quaterniond& data);

		/**
		 * @brief Reads a pose
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param dwl::Pose& data Data to read
		 */
		void read(const YAML::Node& node, Pose& data);

		/**
		 * @brief Reads a 3d pose
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param dwl::Pose3d& data Data to read
		 */
		void read(const YAML::Node& node, Pose3d& data);

		/**
		 * @brief Reads a 3d action
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param dwl::Action3d& data Data to read
		 */
		void read(const YAML::Node& node, Action3d& data);

		/**
		 * @brief Reads a search area
		 * @param const YAML::Node& node Namespace where is defined the data
		 * @param dwl::SearchArea& data Data to read
		 */
		void read(const YAML::Node& node, SearchArea& data);
};

} //@namespace dwl
