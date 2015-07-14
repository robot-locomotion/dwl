#include <solver/Solver.h>
#include <solver/IpoptNLP.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
	dwl::solver::Solver* solver = new dwl::solver::IpoptNLP();

	dwl::model::OptimizationModel model;
	dwl::model::DynamicalSystem* dynamic_system = new dwl::model::HS071DynamicalSystem();
	dwl::model::Cost* cost = new dwl::model::HS071Cost();

	model.addDynamicSystem(dynamic_system);
	model.addCost(cost);

	solver->setModel(&model);

//	solver->addCost(&cost);
//	solver->addConstraint(&constraint);

//	planning_ptr_->reset(&robot_, solver_, environment_);

	solver->init();
	solver->compute();

	return 0;
}
