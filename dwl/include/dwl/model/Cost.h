#ifndef DWL__MODEL__COST__H
#define DWL__MODEL__COST__H

#include <dwl/environment/RewardMap.h>
#include <dwl/utils/utils.h>


namespace dwl
{

namespace model
{

struct CostVariables
{
	CostVariables(bool full_opt = false) : time(full_opt), base_pos(full_opt), base_vel(full_opt),
			base_acc(full_opt), base_eff(full_opt), joint_pos(full_opt), joint_vel(full_opt),
			joint_acc(full_opt), joint_eff(full_opt), contact_pos(full_opt), contact_vel(full_opt),
			contact_acc(full_opt), contact_for(full_opt) {}
	bool time;
	bool base_pos;
	bool base_vel;
	bool base_acc;
	bool base_eff;
	bool joint_pos;
	bool joint_vel;
	bool joint_acc;
	bool joint_eff;
	bool contact_pos;
	bool contact_vel;
	bool contact_acc;
	bool contact_for;
};
/**
 * @class Cost
 * @brief Abstract class for computing the cost function in optimization-based locomotion (planning
 * or control) approach
 */
class Cost
{
	public:
		/** @brief Constructor function */
		Cost();

		/** @brief Destructor function */
		virtual ~Cost();

		/**
		 * @brief Computes the cost value given a certain state
		 * @param double& Cost value
		 * @param const WholeBodyState& Whole-body state
		 */
		virtual void compute(double& cost,
							 const WholeBodyState& state) = 0;

		/**
		 * @brief Sets the whole-body state weights which are used by specific cost function
		 * @param WholeBodyState& Whole-body weights
		 */
		void setWeights(const WholeBodyState& weights);

		/**
		 * @brief Sets the desired whole-body state
		 * @param const WholeBodyState& Desired whole-body state
		 */
		void setDesiredState(const WholeBodyState& desired_state);

		/**
		 * @brief Gets the name of the cost
		 * @return The name of the cost
		 */
		std::string getName();


	protected:
		/** @brief Name of the cost */
		std::string name_;

		/** @brief Cost variables defines by the locomotion weights */
		CostVariables cost_variables_;

		/** @brief Whole-body state weights */
		WholeBodyState locomotion_weights_;

		/** @brief Desired whole-body state */
		WholeBodyState desired_state_;
};

} //@namespace model
} //@namespace dwl

#endif
