#ifndef DWL__OCP__CONSTRAINT__H
#define DWL__OCP__CONSTRAINT__H

#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/utils/URDF.h>
#include <dwl/utils/utils.h>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#define NO_BOUND 2e19


namespace dwl
{

namespace ocp
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
		 * @param std::string Semantic system description filename
		 * @param Print model information
		 */
		void modelFromURDFFile(std::string urdf_file,
							   std::string system_file = std::string(),
							   bool info = false);

		/**
		 * @brief Build the model rigid-body system from an URDF model (xml)
		 * @param std::string URDF model
		 * @param std::string Semantic system description filename
		 * @param Print model information
		 */
		void modelFromURDFModel(std::string urdf_model,
								std::string system_file = std::string(),
								bool info = false);

		/** @brief Sets the constraint as soft constraint, i.e. inside the cost function */
		void defineAsSoftConstraint();

		/** @brief Sets the constraint as hard constraint, i.e. inside the cost function */
		void defineAsHardConstraint();

		/**
		 * @brief Initializes the constraint properties given an URDF model (xml)
		 * @param Print model information
		 */
		virtual void init(bool info = false);

		/**
		 * @brief Computes the soft-value of the constraint given a certain state
		 * @param double& Soft-value or the associated cost to the constraint
		 * @param const WholeBodyState& Whole-body state
		 */
		void computeSoft(double& constraint_cost,
						 const WholeBodyState& state);

		/**
		 * @brief Computes the constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		virtual void compute(Eigen::VectorXd& constraint,
							 const WholeBodyState& state) = 0;

		/**
		 * @brief Gets the lower and upper bounds of the constraint
		 * @param Eigen::VectorXd& Lower constraint bound
		 * @param Eigen::VectorXd& Upper constraint bound
		 */
		virtual void getBounds(Eigen::VectorXd& lower_bound,
							   Eigen::VectorXd& upper_bound) = 0;

		/** @brief Indicates is the constraint is implemented as soft-constraint */
		bool isSoftConstraint();

		/**
		 * @brief Sets the last state that could be used for the constraint
		 * @param WholeBodyState& Last whole-body state
		 */
		void setLastState(WholeBodyState& last_state);

		/** @brief Resets the state buffer */
		void resetStateBuffer();

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

		/** @brief Label that indicates if it's implemented as soft constraint */
		bool is_soft_;

		/** @brief Sets the last state */
		boost::circular_buffer<WholeBodyState> state_buffer_;

		/** @brief A floating-base system definition */
		model::FloatingBaseSystem system_;

		/** @brief Whole-body kinematical model */
		model::WholeBodyKinematics kinematics_;

		/** @brief Whole-body dynamical model */
		model::WholeBodyDynamics dynamics_;
};

} //@namespace ocp
} //@namespace dwl

#endif
