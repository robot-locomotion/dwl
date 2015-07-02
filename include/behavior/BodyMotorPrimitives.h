#ifndef DWL_BodyMotorPrimitives_H
#define DWL_BodyMotorPrimitives_H

#include <behavior/MotorPrimitives.h>
#include <utils/utils.h>
#include <utils/YamlBridge.h>


namespace dwl
{

namespace behavior
{

struct BodyMotorPrimitive
{
		Eigen::Vector3d action;
		double cost;
};

/**
 * @class BodyMotorPrimitives
 * @brief Class for generating body motor primitives
 */
class BodyMotorPrimitives : public MotorPrimitives
{
	public:
		/** @brief Constructor function */
		BodyMotorPrimitives();

		/** @brief Destructor function */
		~BodyMotorPrimitives();

		/**
		 * @brief Reads the information of the motor primitives from a file path
		 * @param std::string File path
		 */
		void read(std::string filepath);

		/**
		 * @brief Abstract method for generation 3D actions
		 * @param std::vector<Action3d>& Set of actions
		 * @param Action3d Current 3D pose
		 */
		void generateActions(std::vector<Action3d>& actions, Pose3d state);

	private:
		/** @brief Vector of body actions */
		std::vector<BodyMotorPrimitive> actions_;

};

} //@namespace behavior
} //@namespace dwl

#endif
