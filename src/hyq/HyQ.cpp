#include <hyq/HyQ.h>


namespace dwl
{

namespace robot
{


HyQ::HyQ()
{

}


HyQ::~HyQ()
{

}


void HyQ::setStanceAreas(std::vector<SearchArea> stance_areas)
{
	SearchArea area;
	double x_foot[] = {0.4269, 0.4269, -0.4269, -0.4269};
	double y_foot[] = {0.3886, -0.3886, 0.3886, -0.3886};
	area.grid_resolution = 0.04;

	// Defining the search areas for the stance position of HyQ
	for (int i = 0; i < 4; i++) {
		area.max_x = x_foot[i] + 0.1;
		area.min_x = x_foot[i] - 0.1;
		area.max_y = y_foot[i] + 0.1;
		area.min_y = y_foot[i] - 0.1;
		stance_areas_.push_back(area);
	}
}


} //@namespace robot

} //@namespace dwl
