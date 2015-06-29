#ifndef DWL_MotorPrimitives_H
#define DWL_MotorPrimitives_H

#include <utils/utils.h>
#include <fstream>
#include <yaml-cpp/yaml.h>


namespace dwl
{

namespace behavior
{

/**
 * @class MotorPrimitives
 * @brief Abstract class for generating motor primitives
 */
class MotorPrimitives
{
	public:
		/** @brief Constructor function */
		MotorPrimitives();

		/** @brief Destructor function */
		virtual ~MotorPrimitives();

		/**
		 * @brief Abstract method for reading a set of motor primitives from the disk
		 * @param std::string Name of the file
		 */
		virtual void read(std::string filepath) = 0;

		/**
		 * @brief Abstract method for generation 3D actions
		 * @param std::vector<Action3d>& Set of actions
		 * @param Action3d Current 3D pose
		 */
		virtual void generateActions(std::vector<Action3d>& actions, Pose3d state);


	protected:
		bool is_defined_motor_primitives_;
};

} //@namespace behavior
} //@namespace dwl

#endif
