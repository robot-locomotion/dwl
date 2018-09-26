#include <dwl/model/AdjacencyModel.h>


namespace dwl
{

namespace model
{

AdjacencyModel::AdjacencyModel() : is_lattice_(false)
{

}


AdjacencyModel::~AdjacencyModel()
{

}


void AdjacencyModel::computeAdjacencyMap(AdjacencyMap& adjacency_map,
										 Vertex source,
										 Vertex target)
{
	printf(YELLOW "Could compute the whole adjacency map because it was not"
			" defined an adjacency model\n" COLOR_RESET);
}


bool AdjacencyModel::isFreeOfObstacle(Vertex state_vertex,
									  TypeOfState state_representation,
									  bool body)
{
	return true;
}


bool AdjacencyModel::isLatticeRepresentation()
{
	return is_lattice_;
}


std::string AdjacencyModel::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
