#ifndef DWL__YAML_WRAPPER__H
#define DWL__YAML_WRAPPER__H

#include <dwl/utils/utils.h>
#include <fstream>
#include <yaml-cpp/yaml.h>


namespace dwl
{

typedef std::vector<std::string> YamlNamespace;

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
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(bool& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a boolean
		 * @param bool& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(bool& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a integer
		 * @param int& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(int& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a integer
		 * @param int& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(int& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a double
		 * @param double& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(double& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a double
		 * @param double& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(double& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a string
		 * @param std::string& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(std::string& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a string
		 * @param std::string& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(std::string& data,
				  const std::string& field,
				  const YAML::Node& node);
		/**
		 * @brief Reads a std vector of doubles
		 * @param std::vector<double>& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<double>& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a std vector of doubles
		 * @param std::vector<double>& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<double>& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a std vector of string
		 * @param std::vector<std::string>& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<std::string>& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a std vector of string
		 * @param std::vector<std::string>& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(std::vector<std::string>& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a 2d vector
		 * @param Eigen::Vector2d& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector2d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a 2d vector
		 * @param Eigen::Vector2d& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector2d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a 3d vector
		 * @param Eigen::Vector3d& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector3d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a 3d vector
		 * @param Eigen::Vector3d& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Vector3d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a quaternion
		 * @param Eigen::Quaterniond& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Quaterniond& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a quaternion
		 * @param Eigen::Quaterniond& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(Eigen::Quaterniond& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a pose
		 * @param Pose& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Pose& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a pose
		 * @param Pose& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(Pose& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a 3d pose
		 * @param Pose3d& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Pose3d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a 3d pose
		 * @param Pose3d& Data to read it
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(Pose3d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a 3d action
		 * @param Action3d& Data to read
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(Action3d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a 3d action
		 * @param Action3d& Data to read
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(Action3d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/**
		 * @brief Reads a search area
		 * @param SearchArea& Data to read
		 * @param const std::string& The name of the field
		 * @param const YamlNamespace& The sequence of namespace
		 * @return Returns true if it was read the data
		 */
		bool read(SearchArea& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/**
		 * @brief Reads a search area
		 * @param SearchArea& Data to read
		 * @param const std::string& The name of the field
		 * @param const YAML::Node& The Yaml node
		 * @return Returns true if it was read the data
		 */
		bool read(SearchArea& data,
				  const std::string& field,
				  const YAML::Node& node);
		/**
		 * @brief Finds the Yaml node given a sequence of namespaces
		 * @param YAML::Node& Yaml node
		 * @param YamlNamespace Namespaces
		 */
		bool getNode(YAML::Node& node,
					 const YamlNamespace& ns = YamlNamespace());


	private:
		/** @brief File name for data reading/writing from/to a yaml */
		std::string filename_;

		/** @brief Labels that indicates that the filename was defined */
		bool is_file_;
};

} //@namespace dwl


inline YAML::Emitter& operator<<(YAML::Emitter& out,
								const Eigen::Vector2d& data)
{
    out << YAML::Flow;
    out << YAML::BeginSeq << data(0) << data(1) << YAML::EndSeq;
    return out;
}

inline YAML::Emitter& operator<<(YAML::Emitter& out,
								const Eigen::Vector3d& data)
{
    out << YAML::Flow;
    out << YAML::BeginSeq << data(0) << data(1) << data(2) << YAML::EndSeq;
    return out;
}

#endif
