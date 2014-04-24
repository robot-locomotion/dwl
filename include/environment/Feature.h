#ifndef DWL_Feature_H
#define DWL_Feature_H

#include <string>


namespace dwl
{

namespace environment
{

class Feature
{
	public:
		Feature();
		virtual ~Feature();

		virtual void compute() = 0;

		std::string getName();

	protected:
		std::string name_;

}; //@class Feature

} //@namespace environment

} //@namespace dwl


#endif
