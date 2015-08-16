#ifndef DWL__MODEL__CONSTRAINT__H
#define DWL__MODEL__CONSTRAINT__H

#include <model/WholeBodyKinematics.h>
#include <model/WholeBodyDynamics.h>
#include <utils/URDF.h>
#include <utils/utils.h>
#define NO_BOUND 2e19


namespace dwl
{

namespace model
{

/**
 * @class Constraint
 * @brief Abstract class for defining constraints in the planning of motion sequences problem
 */
class Constraint
{
	public:
		/** @brief Constructor function */
		Constraint();

		/** @brief Destructor function */
		virtual ~Constraint();

		/**
		 * @brief Build the model rigid-body system from an URDF file
		 * @param std::string URDF file
		 * @param Print model information
		 */
		void modelFromURDFFile(std::string urdf_model,
							   bool info = false);

		/**
		 * @brief Build the model rigid-body system from an URDF model (xml)
		 * @param std::string URDF model
		 * @param Print model information
		 */
		void modelFromURDFModel(std::string urdf_model,
								bool info = false);

		/**
		 * @brief Initializes the constraint properties given an URDF model (xml)
		 * @param std::string URDF model
		 * @param Print model information
		 */
		virtual void init(std::string urdf_model,
						  bool info = false);

		/**
		 * @brief Computes the constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void compute(Eigen::VectorXd& constraint,
							 const LocomotionState& state) = 0;

		/**
		 * @brief Gets the lower and upper bounds of the constraint
		 * @param Eigen::VectorXd& Lower constraint bound
		 * @param Eigen::VectorXd& Upper constraint bound
		 */
		virtual void getBounds(Eigen::VectorXd& lower_bound,
							   Eigen::VectorXd& upper_bound) = 0;

		/**
		 * @brief Sets the last state that could be used for the constraint
		 * @param LocomotionState& Last state
		 */
		void setLastState(LocomotionState& last_state);

		/** @brief Gets the dimension of the constraint */
		unsigned int getConstraintDimension();

		/**
		 * @brief Gets the name of the constraint
		 * @return The name of the constraint
		 */
		std::string getName();


	protected:
		/** @brief Name of the constraint */
		std::string name_;

		/** @brief Dimension of the constraint */
		unsigned int constraint_dimension_;

		/** @brief Sets the last state */
		LocomotionState last_state_;

		/** @brief A floating-base system definition */
		FloatingBaseSystem system_;

		/** @brief Whole-body kinematical model */
		WholeBodyKinematics kinematics_;

		/** @brief Whole-body dynamical model */
		WholeBodyDynamics dynamics_;
};

} //@namespace model
} //@namespace dwl

#endif
