#ifndef DWL_Model_H
#define DWL_Model_H

#include <model/Constraint.h>
#include <model/Cost.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{


class Model
{
	public:
		Model();
		~Model();

		/**
		 * @brief Adds an active or inactive constraints to the planning algorithm
		 * @param Constraint* Constraint to add it
		 */
		void addConstraint(Constraint* constraint);

		/**
		 * @brief Removes an active or inactive constraints to the planning algorithm
		 * @param Constraint* Constraint to remove it
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost function for the planning algorithm
		 * @param Cost* Cost to add it
		 */
		void addCost(Cost* cost);

		/**
		 * @brief Removes a cost function for the planning algorithm
		 * @param Cost* Cost to remove it
		 */
		void removeCost(std::string cost_name);

		std::vector<Constraint*> getActiveConstraints();
		std::vector<Constraint*> getInactiveConstraints();
		std::vector<Cost*> getCosts();

		void convertDecisionVariablesToStateModel(StateModel& state_model,
												  const Eigen::VectorXd& variables); //TODO make virtual

		void setDimensionOfDecisionVariables(); //TODO make virtual
		int getDimensionOfDecisionVariables();
		int getDimensionOfConstraints();


	private:
		/** @brief Vector of active constraints pointers */
		std::vector<Constraint*> active_constraints_;

		/** @brief Vector of inactive constraints pointers */
		std::vector<Constraint*> inactive_constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		int state_dimension_;
		int constraint_dimension_;

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_active_constraint_;

		/** @brief Indicates if it was added an inactive constraint in the solver */
		bool is_added_inactive_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;
};

} //@namespace model
} //@namespace dwl


#endif
