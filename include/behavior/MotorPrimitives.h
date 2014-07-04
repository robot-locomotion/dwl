#ifndef DWL_MotorPrimitives_H
#define DWL_MotorPrimitives_H

#include <utils/utils.h>


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
		 * @brief Abstract method for generation 3D actions
		 * @param std::vector<dwl::Action3d>& actions Set of actions
		 * @param dwl::A3d state Current 3D pose
		 */
		virtual void generateActions(std::vector<Action3d>& actions, Pose3d state);
};

} //@namespace behavior
} //@namespace dwl

#endif
