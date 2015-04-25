#include <model/Model.h>


namespace dwl
{

namespace model
{

Model::Model() : state_dimension_(0), constraint_dimension_(0),
		is_added_active_constraint_(false), is_added_inactive_constraint_(false),
		is_added_cost_(false)
{

}

Model::~Model()
{
	typedef std::vector<model::Constraint*>::iterator ConstraintItr;
	typedef std::vector<model::Cost*>::iterator CostItr;
	if (is_added_active_constraint_) {
		for (ConstraintItr i = active_constraints_.begin(); i != active_constraints_.end(); i++)
			delete *i;
	}
	if (is_added_inactive_constraint_) {
		for (ConstraintItr i = inactive_constraints_.begin(); i != inactive_constraints_.end(); i++)
			delete *i;
	}
	if (is_added_cost_) {
		for (CostItr i = costs_.begin(); i != costs_.end(); i++)
			delete *i;
	}
}


void Model::addConstraint(model::Constraint* constraint)
{
	if (constraint->isActive()) {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		active_constraints_.push_back(constraint);

		if (!is_added_active_constraint_)
			is_added_active_constraint_ = true;
	}
	else {
		printf(GREEN "Adding the inactive %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		inactive_constraints_.push_back(constraint);

		if (!is_added_inactive_constraint_)
			is_added_inactive_constraint_ = true;
	}

	// Updating the constraint dimension
	constraint_dimension_ += constraint->getConstraintDimension();
}


void Model::removeConstraint(std::string constraint_name)
{
	int max_num_constraints;
	if (is_added_active_constraint_ & is_added_inactive_constraint_)
		max_num_constraints = (active_constraints_.size() > inactive_constraints_.size()) ?
				active_constraints_.size() : inactive_constraints_.size();
	else if (is_added_active_constraint_)
		max_num_constraints = active_constraints_.size();
	else if (is_added_inactive_constraint_)
		max_num_constraints = inactive_constraints_.size();
	else {
		printf(YELLOW "Could not removed the %s constraint because has not been added an constraint\n" COLOR_RESET,
				constraint_name.c_str());

		return;
	}

	if (max_num_constraints == 0)
		printf(YELLOW "Could not removed the %s constraint because there is not constraints\n" COLOR_RESET,
				constraint_name.c_str());
	else {
		for (int i = 0; i < max_num_constraints; i++) {
			if (is_added_active_constraint_) {
				if (i < (int) active_constraints_.size()) {
					if (constraint_name == active_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the active %s constraint\n" COLOR_RESET,
								active_constraints_[i]->getName().c_str());

						// Updating the constraint dimension
						constraint_dimension_ -= active_constraints_[i]->getConstraintDimension();

						// Deleting the constraint
						delete active_constraints_.at(i);
						active_constraints_.erase(active_constraints_.begin() + i);

						return;
					}
				}
			}

			if (is_added_inactive_constraint_) {
				if (i < (int) inactive_constraints_.size()) {
					if (constraint_name == inactive_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the inactive %s constraint\n" COLOR_RESET,
								inactive_constraints_[i]->getName().c_str());

						// Updating the constraint dimension
						constraint_dimension_ -= inactive_constraints_[i]->getConstraintDimension();

						// Deleting the constraint
						delete inactive_constraints_.at(i);
						inactive_constraints_.erase(inactive_constraints_.begin() + i);

						return;
					}
				}
			}

			if (i == max_num_constraints - 1) {
				printf(YELLOW "Could not removed the %s constraint\n" COLOR_RESET,
						constraint_name.c_str());
			}
		}
	}
}


void Model::addCost(model::Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());

	costs_.push_back(cost);
	is_added_cost_ = true;
}


void Model::removeCost(std::string cost_name)
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


std::vector<model::Constraint*> Model::getActiveConstraints()
{
	return active_constraints_;
}


std::vector<model::Constraint*> Model::getInactiveConstraints()
{
	return inactive_constraints_;
}


std::vector<model::Cost*> Model::getCosts()
{
	return costs_;
}


void Model::convertDecisionVariablesToStateModel(StateModel& state_model,
												 const Eigen::VectorXd& decision_var)
{
	state_dimension_ = 4;
	state_model.base_pos(0) = decision_var(0);
	state_model.base_pos(1) = decision_var(1);
	state_model.base_pos(2) = decision_var(2);
	state_model.base_pos(3) = decision_var(3);
}


int Model::getDimensionOfDecisionVariables()
{
	return state_dimension_;
}


int Model::getDimensionOfConstraints()
{
	return constraint_dimension_;
}

} //@namespace model
} //@namespace dwl
