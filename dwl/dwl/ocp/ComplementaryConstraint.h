#ifndef DWL__OCP__COMPLEMENTARY_CONSTRAINT__H
#define DWL__OCP__COMPLEMENTARY_CONSTRAINT__H

#include <dwl/ocp/Constraint.h>


namespace dwl
{

namespace ocp
{

class ComplementaryConstraint : public Constraint<WholeBodyState>
{
	public:
		/** @brief Constructor function */
		ComplementaryConstraint();

		/** @brief Destructor function */
		virtual ~ComplementaryConstraint();

		/** @brief Resets the robot dynamics
		 * @param[in] fbs Floating-base model
		 * @param[in] wdyn Whole-body dynamics
		 **/
		void reset(model::FloatingBaseSystem& fbs,
				   model::WholeBodyKinematics& wkin,
				   model::WholeBodyDynamics& wdyn);

		/**
		 * @brief Initializes the complementary constraint properties given an URDF model (xml)
		 * @param std::string URDF model
		 * @param Print model information
		 */
		virtual void init(std::string urdf_model,
						  bool info);

		/**
		 * @brief Computes the complementary constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		void compute(Eigen::VectorXd& constraint,
					 const WholeBodyState& state);

		/**
		 * @brief Gets the bounds of the complementary constraints
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);

		/**
		 * @brief Computes the first complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		virtual void computeFirstComplement(Eigen::VectorXd& constraint,
											const WholeBodyState& state) = 0;

		/**
		 * @brief Computes the second complement constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		virtual void computeSecondComplement(Eigen::VectorXd& constraint,
											 const WholeBodyState& state) = 0;


	protected:
		/** @brief Dimension of the complementary constraints */
		unsigned int complementary_dimension_;

		/** @brief Floating-base system */
		std::shared_ptr<model::FloatingBaseSystem> fbs_;

		/** @brief Whole-body kinematics */
		std::shared_ptr<model::WholeBodyKinematics> wkin_;

		/** @brief Whole-body dynamics */
		std::shared_ptr<model::WholeBodyDynamics> wdyn_;
};

} //@namespace ocp
} //@namespace dwl

#endif
