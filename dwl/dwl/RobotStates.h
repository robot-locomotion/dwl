#ifndef DWL__ROBOT_STATES__H
#define DWL__ROBOT_STATES__H

#include <dwl/WholeBodyState.h>
#include <dwl/ReducedBodyState.h>
#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/utils/FrameTF.h>


namespace dwl
{

class RobotStates
{
	public:
		/** @brief Constructor function */
		RobotStates();

		/** @brief Destructor function */
		~RobotStates();

		/** @brief Resets the robot dynamics
		 * @param[in] fbs Floating-base model
		 * @param[in] wdyn Whole-body dynamics
		 **/
		void reset(model::FloatingBaseSystem& fbs,
				   model::WholeBodyKinematics& wkin,
				   model::WholeBodyDynamics& wdyn);

		/** @brief Set the force threshold for getting active contacts */
		void setForceThreshold(double force_threshold);

		/**
		 * @brief Converts the reduced-body state to whole-body one
		 * @param const ReducedBodyStated& Reduced-body state
		 * @return const WholeBodyState& Whole-body state
		 */
		const WholeBodyState& getWholeBodyState(const ReducedBodyState& state);

		/**
		 * @brief Converts the whole-body state to reduced-body state one
		 * @param const WholeBodyState& Whole-body state
		 * @return const ReducedBodyStated& Reduced-body state
		 */
		const ReducedBodyState& getReducedBodyState(const WholeBodyState& state);

		/**
		 * @brief Converts a reduced-body trajectory to a whole-body one
		 * @param const ReducedBodyTrajectory& Reduced-body trajectory
		 * @return const WholeBodyTrajectory& Whole-body trajectory
		 */
		const WholeBodyTrajectory& getWholeBodyTrajectory(const ReducedBodyTrajectory& trajectory);
		const ReducedBodyTrajectory& getReducedBodyTrajectory(const WholeBodyTrajectory& trajectory);


	private:
		/**
		 * @brief Computes the base velocity in the world frame from the
		 * CoM acceleration
		 * @param const ReducedBodyState& Reduced state
		 * @param const Eigen::Vector3d& CoM fixed displacement expressed in
		 * the world frame
		 */
		Eigen::Vector3d computeBaseVelocity_W(const ReducedBodyState& state,
											  const Eigen::Vector3d& com_pos_W);

		/**
		 * @brief Computes the base acceleration in the world frame from the
		 * CoM acceleration
		 * @param const ReducedBodyState& Reduced state
		 * @param const Eigen::Vector3d& CoM fixed displacement expressed in
		 * the world frame
		 */
		Eigen::Vector3d computeBaseAcceleration_W(const ReducedBodyState& state,
												  const Eigen::Vector3d& com_pos_W);

		/** @brief Whole-body state */
		WholeBodyState ws_;

		/** @brief Reduced-body state */
		ReducedBodyState rs_;

		/** @brief Whole-body trajectory */
		WholeBodyTrajectory wt_;

		/** @brief Reduced-body trajectory */
		ReducedBodyTrajectory rt_;

		/** @brief Floating-base system */
		std::shared_ptr<model::FloatingBaseSystem> fbs_;

		/** @brief Whole-body kinematics */
		std::shared_ptr<model::WholeBodyKinematics> wkin_;

		/** @brief Whole-body dynamics */
		std::shared_ptr<model::WholeBodyDynamics> wdyn_;

		/** @brief Frame transformer */
		math::FrameTF frame_tf_;

		/** @brief Robot properties */
		Eigen::Vector3d com_pos_B_;
		unsigned int num_joints_;
		unsigned int num_feet_;
		rbd::BodySelector feet_;

		/** @brief Force threshold */
		double force_threshold_;
};

} //@namespace


#endif
