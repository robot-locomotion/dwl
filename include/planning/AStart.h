#ifndef DWL_AStart_H
#define DWL_AStart_H

#include <planning/Solver.h>
#include <environment/PlaneGrid.h>


namespace dwl
{

namespace planning
{

class AStart : public dwl::planning::Solver
{
	public:
		AStart();
		~AStart();

		virtual bool init();

		virtual bool compute(SolverInterface solver_interface);

		void findShortestPath(Vertex source, Vertex target, AdjacencyMap adjacency_map, VertexCost& min_cost, PreviousVertex& previous);

		double heuristicCostEstimate(Vertex source, Vertex target);


	private:



};

} //@namespace planning

} //@namespace dwl


#endif
