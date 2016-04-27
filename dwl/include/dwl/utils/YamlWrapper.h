#ifndef DWL__YAML_WRAPPER__H
#define DWL__YAML_WRAPPER__H

#include <dwl/utils/utils.h>
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
		 * @brief Reads a boolean
		 * @param bool& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(bool& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a integer
		 * @param int& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(int& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a double
		 * @param double& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(double& data,
				  const YAML::Node& node,
				  std::string field);


		/**
		 * @brief Reads a string
		 * @param std::string& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(std::string& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a std vector of doubles
		 * @param std::vector<double>& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<double>& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a std vector of string
		 * @param std::vector<string>& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<std::string>& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a 2d vector
		 * @param Eigen::Vector2d& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector2d& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a 3d vector
		 * @param Eigen::Vector3d& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector3d& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a quaternion
		 * @param Eigen::Quaterniond& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Quaterniond& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a pose
		 * @param Pose& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(Pose& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a 3d pose
		 * @param Pose3d& Data to read it
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(Pose3d& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a 3d action
		 * @param Action3d& Data to read
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(Action3d& data,
				  const YAML::Node& node,
				  std::string field);

		/**
		 * @brief Reads a search area
		 * @param SearchArea& Data to read
		 * @param const YAML::Node& Namespace where is defined the data
		 * @param std::string The name of the field
		 * @return Returns true if it was read the data
		 */
		bool read(SearchArea& data,
				  const YAML::Node& node,
				  std::string field);
};

} //@namespace dwl

#endif
