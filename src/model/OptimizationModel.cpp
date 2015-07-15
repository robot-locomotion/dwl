#include <model/OptimizationModel.h>


namespace dwl
{

namespace model
{

OptimizationModel::OptimizationModel() : dynamical_system_(NULL),
		state_dimension_(0), constraint_dimension_(0), horizon_(1),
		is_added_dynamic_system_(false), is_added_constraint_(false),
		is_added_cost_(false)
{

}


OptimizationModel::~OptimizationModel()
{
	typedef std::vector<Constraint*>::iterator ConstraintItr;
	typedef std::vector<Cost*>::iterator CostItr;
	if (is_added_constraint_) {
		for (ConstraintItr i = constraints_.begin(); i != constraints_.end(); i++)
			delete *i;
	}

	if (is_added_cost_) {
		for (CostItr i = costs_.begin(); i != costs_.end(); i++)
			delete *i;
	}
}


void OptimizationModel::addDynamicSystem(DynamicalSystem* dynamical_system)
{
	if (is_added_dynamic_system_) {
		printf(YELLOW "Could not added two dynamic systems\n" COLOR_RESET);
		return;
	}

	printf(GREEN "Adding the %s dynamic system\n" COLOR_RESET, dynamical_system->getName().c_str());
	dynamical_system_ = dynamical_system;

	// Reading the state dimension
	state_dimension_ = getDynamicalSystem()->getDimensionOfState();

	// Updating the constraint dimension
	constraint_dimension_ += dynamical_system->getConstraintDimension();
}


void OptimizationModel::removeDynamicSystem()
{
	if (is_added_dynamic_system_) {
		dynamical_system_ = NULL;

		// Updating the constraint dimension
		constraint_dimension_ -= dynamical_system_->getConstraintDimension();
	} else
		printf(YELLOW "There was not added a dynamic systems\n" COLOR_RESET);
}


void OptimizationModel::addConstraint(Constraint* constraint)
{
	printf(GREEN "Adding the %s dynamic system constraint\n" COLOR_RESET, constraint->getName().c_str());
	constraints_.push_back(constraint);

	if (!is_added_constraint_)
		is_added_constraint_ = true;

	// Updating the constraint dimension
	constraint_dimension_ += constraint->getConstraintDimension();
}


void OptimizationModel::removeConstraint(std::string constraint_name)
{
	if (is_added_constraint_) {
		unsigned int num_constraints = constraints_.size();
		for (unsigned int i = 0; i < num_constraints; i++) {
			if (constraint_name == constraints_[i]->getName().c_str()) {
				printf(GREEN "Removing the %s constraint\n" COLOR_RESET, constraints_[i]->getName().c_str());

				// Updating the constraint dimension
				constraint_dimension_ -= constraints_[i]->getConstraintDimension();

				// Deleting the constraint
				delete constraints_.at(i);
				constraints_.erase(constraints_.begin() + i);

				return;
			}
		}
	} else {
		printf(YELLOW "Could not removed the %s constraint because has not been added an constraint\n" COLOR_RESET,
				constraint_name.c_str());
		return;
	}
}


void OptimizationModel::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());

	costs_.push_back(cost);
	is_added_cost_ = true;
}


void OptimizationModel::removeCost(std::string cost_name)
{
	if (is_added_cost_) {
		if (costs_.size() == 0)
			printf(YELLOW "Could not removed the %s cost because there is not cost\n" COLOR_RESET,
					cost_name.c_str());
		else {
			unsigned int costs_size = costs_.size();
			for (unsigned int i = 0; i < costs_size; i++) {
				if (cost_name == costs_[i]->getName().c_str()) {
					printf(GREEN "Removing the %s cost\n" COLOR_RESET, costs_[i]->getName().c_str());

					// Deleting the cost
					delete costs_.at(i);
					costs_.erase(costs_.begin() + i);

					return;
				}
				else if (i == costs_.size() - 1) {
					printf(YELLOW "Could not removed the %s cost\n" COLOR_RESET, cost_name.c_str());
				}
			}
		}
	}
	else
		printf(YELLOW "Could not removed the %s cost because has not been added an cost\n" COLOR_RESET,
				cost_name.c_str());
}


void OptimizationModel::setHorizon(unsigned int horizon)
{
	if (horizon == 0)
		horizon_ = 1;
	else
		horizon_ = horizon;
}


DynamicalSystem* OptimizationModel::getDynamicalSystem()
{
	return dynamical_system_;
}


std::vector<Constraint*> OptimizationModel::getConstraints()
{
	return constraints_;
}


std::vector<Cost*> OptimizationModel::getCosts()
{
	return costs_;
}


unsigned int OptimizationModel::getDimensionOfState()
{
	return state_dimension_ * horizon_;
}


unsigned int OptimizationModel::getDimensionOfConstraints()
{
	return constraint_dimension_ * horizon_;
}


unsigned int OptimizationModel::getHorizon()
{
	return horizon_;
}

} //@namespace model
} //@namespace dwl
