#ifndef DWL_BodyMotorPrimitives_H
#define DWL_BodyMotorPrimitives_H

#include <behavior/MotorPrimitives.h>
#include <utils/utils.h>


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
		 * @brief Abstract method for generation 3D actions
		 * @param std::vector<dwl::Action3d>& actions Set of actions
		 * @param dwl::Action3d state Current 3D pose
		 */
		void generateActions(std::vector<Action3d>& actions, Pose3d state);


	private:
		std::vector<BodyMotorPrimitive> actions_;

};

} //@namespace behavior
} //@namespace dwl

#endif
