#ifndef DWL_BodyMotorPrimitives_H
#define DWL_BodyMotorPrimitives_H

#include <behavior/MotorPrimitives.h>
#include <utils/utils.h>


namespace dwl
{

namespace behavior
{

class BodyMotorPrimitives : public MotorPrimitives
{
	public:
		BodyMotorPrimitives();
		~BodyMotorPrimitives();

		void generateActions(std::vector<Pose3d>& actions, Pose3d state);

	private:

};

} //@namespace behavior
} //@namespace dwl

#endif
