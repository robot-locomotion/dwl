#ifndef DWL_MotorPrimitives_H
#define DWL_MotorPrimitives_H

#include <utils/utils.h>


namespace dwl
{

namespace behavior
{

class MotorPrimitives
{
	public:
		MotorPrimitives();
		virtual ~MotorPrimitives();

		virtual void generateActions(std::vector<Pose3d>& actions, Pose3d state);
};

} //@namespace behavior
} //@namespace dwl

#endif
