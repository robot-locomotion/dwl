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
		/** @brief Constructor functions */
		YamlWrapper();
		YamlWrapper(std::string filename);

		/** @brief Destructor function */
		~YamlWrapper();

		/**
		 * @brief Sets the filename for reading/writing
		 * @param std::string File name
		 */
		void setFile(std::string filename);

		/**
		 * @brief Reads a boolean
		 * @param bool& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(bool& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(bool& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a integer
		 * @param int& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(int& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(int& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a double
		 * @param double& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(double& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(double& data,
				  const YAML::Node& node,
				  std::string field) {return false;};


		/**
		 * @brief Reads a string
		 * @param std::string& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(std::string& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(std::string& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a std vector of doubles
		 * @param std::vector<double>& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<double>& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(std::vector<double>& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a std vector of string
		 * @param std::vector<string>& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<std::string>& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(std::vector<std::string>& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a 2d vector
		 * @param Eigen::Vector2d& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector2d& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(Eigen::Vector2d& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a 3d vector
		 * @param Eigen::Vector3d& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector3d& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(Eigen::Vector3d& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a quaternion
		 * @param Eigen::Quaterniond& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Quaterniond& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(Eigen::Quaterniond& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a pose
		 * @param Pose& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Pose& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(Pose& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a 3d pose
		 * @param Pose3d& Data to read it
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Pose3d& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(Pose3d& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a 3d action
		 * @param Action3d& Data to read
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Action3d& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(Action3d& data,
				  const YAML::Node& node,
				  std::string field) {return false;};

		/**
		 * @brief Reads a search area
		 * @param SearchArea& Data to read
		 * @param std::string The name of the field
		 * @param std::vector<std::string> The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(SearchArea& data,
				  std::string field,
				  std::vector<std::string> ns = std::vector<std::string>());
		bool read(SearchArea& data,
				  const YAML::Node& node,
				  std::string field) {return false;};
	private:
		/**
		 * @brief Finds the Yaml node given a sequence of namespaces
		 * @param YAML::Node& Yaml node
		 * @param std::vector<std::string> Namespaces
		 */
		bool findNode(YAML::Node& node,
					  std::vector<std::string> ns);

		/** @brief File name for data reading/writing from/to a yaml */
		std::string filename_;

		/** @brief Labels that indicates that the filename was defined */
		bool is_file_;
};

} //@namespace dwl

#endif
