#ifndef DWL_BodyMotorPrimitives_H
#define DWL_BodyMotorPrimitives_H

#include <utils/utils.h>


namespace dwl
{

namespace behavior
{

class BodyMotorPrimitives
{
	public:
		BodyMotorPrimitives();
		~BodyMotorPrimitives();

		void computeAction(std::vector<Pose3d>& actions, Pose3d state);

	private:

};

} //@namespace behavior
} //@namespace dwl

#endif
