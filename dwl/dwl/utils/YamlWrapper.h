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
 * @brief This class includes different functions for reading and writing YAML files
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 */
class YamlWrapper
{
	public:
		/** @brief Constructor functions */
		YamlWrapper();
		YamlWrapper(std::string filename);

		/** @brief Destructor function */
		~YamlWrapper();

		/** @brief Sets the filename for reading/writing
		 * @param[in] The YAML file to parse
		 */
		void setFile(std::string filename);

		/** @brief Reads a boolean from YAML file
		 * @param[out] data The value of the read Boolean data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(bool& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a boolean from YAML file
		 * @param[out] data The value of the read Boolean data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(bool& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads an integer from YAML file
		 * @param[out] data The value of the read integer data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(int& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads an integer from YAML file
		 * @param[out] data The value of the read integer data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(int& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a double from YAML file
		 * @param[out] data The value of the read double data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(double& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a double from YAML file
		 * @param[out] data The value of the read double data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(double& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a string from YAML file
		 * @param[out] data The value of the read string data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(std::string& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a string from YAML file
		 * @param[out] data The value of the read string data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(std::string& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a std::vector<double> from YAML file
		 * @param[out] data The value of the read std::vector<double> data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(std::vector<double>& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a std::vector<double> from YAML file
		 * @param[out] data The value of the read std::vector<double> data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(std::vector<double>& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a std::vector<string> from YAML file
		 * @param[out] data The value of the read std::vector<string> data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(std::vector<std::string>& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a std::vector<string> from YAML file
		 * @param[out] data The value of the read std::vector<string> data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(std::vector<std::string>& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a Vector2d from YAML file
		 * @param[out] data The value of the read Vector2d data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */		bool read(Eigen::Vector2d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a Vector2d from YAML file
		 * @param[out] data The value of the read Vector2d data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(Eigen::Vector2d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a Vector3d from YAML file
		 * @param[out] data The value of the read Vector3d data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(Eigen::Vector3d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a Vector3d from YAML file
		 * @param[out] data The value of the read Vector3d data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(Eigen::Vector3d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a Quaternion from YAML file
		 * @param[out] data The value of the read Quaternion data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(Eigen::Quaterniond& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a Quaternion from YAML file
		 * @param[out] data The value of the read Quaternion data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(Eigen::Quaterniond& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a Pose from YAML file
		 * @param[out] data The value of the read Pose data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(Pose& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a Pose from YAML file
		 * @param[out] data The value of the read Pose data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(Pose& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads a Pose3d from YAML file
		 * @param[out] data The value of the read Pose3d data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(Pose3d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads a Pose3d from YAML file
		 * @param[out] data The value of the read Pose3d data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(Pose3d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads an Action3d from YAML file
		 * @param[out] data The value of the read Action3d data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(Action3d& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads an Action3d from YAML file
		 * @param[out] data The value of the read Action3d data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(Action3d& data,
				  const std::string& field,
				  const YAML::Node& node);

		/** @brief Reads an SearchArea from YAML file
		 * @param[out] data The value of the read SearchArea data
		 * @param[in] field The field where it has to be read
		 * @param[in] ns The YAML namespace where it is the date
		 * @return True if it was read the data
		 */
		bool read(SearchArea& data,
				  const std::string& field,
				  const YamlNamespace& ns = YamlNamespace());

		/** @brief Reads an SearchArea from YAML file
		 * @param[out] data The value of the read SearchArea data
		 * @param[in] field The field where it has to be read
		 * @param[in] node The YAML node
		 * @return True if it was read the data
		 */
		bool read(SearchArea& data,
				  const std::string& field,
				  const YAML::Node& node);
		/** @brief Finds the Yaml node given a sequence of namespaces
		 * @param[out] node The output Yaml node
		 * @param[in] The given namespace
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
