#ifndef DWL__YAML_WRAPPER__H
#define DWL__YAML_WRAPPER__H

#include <utils/utils.h>
#include <fstream>
#include <yaml-cpp/yaml.h>


namespace dwl
{

/**
 * @class YamlWrapper
 * @brief Class for reading different yaml structures of the DWL
 */
class YamlWrapper
{
	public:
		/** @brief Constructor function */
		YamlWrapper();

		/** @brief Destructor function */
		~YamlWrapper();

		/**
		 * @brief Reads a double
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param double& Data to read it
		 */

		void read(const YAML::Node& node, double& data);

		/**
		 * @brief Reads a string
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string& Data to read it
		 */
		void read(const YAML::Node& node, std::string& data);

		/**
		 * @brief Reads a std vector of doubles
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::vector<double>& Data to read it
		 */
		void read(const YAML::Node& node, std::vector<double>& data);

		/**
		 * @brief Reads a std vector of string
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::vector<string>& Data to read it
		 */
		void read(const YAML::Node& node, std::vector<std::string>& data);

		/**
		 * @brief Reads a 2d vector
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param Eigen::Vector2d& Data to read it
		 */
		void read(const YAML::Node& node, Eigen::Vector2d& data);

		/**
		 * @brief Reads a 3d vector
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param Eigen::Vector3d& Data to read it
		 */
		void read(const YAML::Node& node, Eigen::Vector3d& data);

		/**
		 * @brief Reads a quaternion
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param Eigen::Quaterniond& Data to read it
		 */
		void read(const YAML::Node& node, Eigen::Quaterniond& data);

		/**
		 * @brief Reads a pose
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param Pose& Data to read it
		 */
		void read(const YAML::Node& node, Pose& data);

		/**
		 * @brief Reads a 3d pose
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param Pose3d& Data to read it
		 */
		void read(const YAML::Node& node, Pose3d& data);

		/**
		 * @brief Reads a 3d action
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param Action3d& Data to read
		 */
		void read(const YAML::Node& node, Action3d& data);

		/**
		 * @brief Reads a search area
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param SearchArea& Data to read
		 */
		void read(const YAML::Node& node, SearchArea& data);
};

} //@namespace dwl

#endif
