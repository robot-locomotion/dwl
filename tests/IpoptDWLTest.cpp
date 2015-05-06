#include <solver/Solver.h>
#include <solver/IpoptNLP.h>
#include <model/HS071Model.cpp>
#include <model/HS071Constraint.cpp>
#include <model/HS071Cost.cpp>



int main(int argc, char **argv)
{
	dwl::solver::Solver* solver = new dwl::solver::IpoptNLP();

	dwl::model::Model* model = new dwl::model::HS071Model();
	dwl::model::Cost* cost = new dwl::model::HS071Cost();
	dwl::model::Constraint* constraint = new dwl::model::HS071Constraint();
	model->addCost(cost);
	model->addConstraint(constraint);

	solver->setModel(model);
//	solver->addCost(&cost);
//	solver->addConstraint(&constraint);

//	planning_ptr_->reset(&robot_, solver_, environment_);
	solver->init();
	solver->compute();

	return 0;
}
