#ifndef DWL__REDUCED_BODY_STATE__H
#define DWL__REDUCED_BODY_STATE__H

#include <Eigen/Dense>
#include <map>
#include <vector>

#include <dwl/utils/FrameTF.h>
#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

/**
 * @brief The ReducedBodyState class
 * This class incorporate the reduced-body state of the robot, including:
 * <ul>
 *    <li>time, in seconds</li>
 *    <li>CoM position, expressed in the world frame of reference</li>
 *    <li>CoM velocity, expressed in the world frame</li>
 *    <li>CoM acceleration, expressed in the world frame</li>
 *    <li>Base orientation, expressed in a world frame of reference</li>
 *    <li>Base rotation rate, expressed in the world frame</li>
 *    <li>Base rotation acceleration, expressed in the world frame</li>
 *    <li>CoP positions, expressed with the world order</li>
 *    <li>Support polygon region, expressed with the world order</li>
 *    <li>Foot positions, expressed in the base frame</li>
 *    <li>Foot velocities, expressed in the base frame</li>
 *    <li>Foot accelerations, expressed in the base frame</li>
 * </ul>
 *
 * The class provides getter and setter methods for all these states.
 * When useful, it provides alternatives to set or get the desired physical
 * quantity in one of the following frames of reference: W (fixed-world frame),
 * B (base frame) and H (horizontal frame). Every getter/setter methods have one
 * of these three letters at the end of the signature, to clearly state what is
 * the reference frame adopted. Also convenient methods to set or get RPY angles
 * are provided. Both Eigen and base types are available for setter methods.
 * You could also interact with the states without using the getter and setter
 * functions. Note that the states are:
 * <ul>
 *   <li>time</li>
 * 	 <li>com_pos [pos_x, pos_y, pos_z]</li>
 * 	 <li>com_vel [vel_x, vel_y, vel_z]</li>
 * 	 <li>com_acc [acc_x, acc_y, acc_z]</li>
 * 	 <li>angular_pos [roll, pitch, yaw]</li>
 * 	 <li>angular_vel [rate_x, rate_y, rate_z]</li>
 * 	 <li>angular_acc [rotacc_x, rotacc_y, rotacc_y]</li>
 * 	 <li>cop [pos_x, pos_y, pos_z]</li>
 * 	 <li>support_region</li>
 * 	 <li>joint_acc</li>
 * 	 <li>joint_eff</li>
 * 	 <li>foot_pos</li>
 * 	 <li>foot_vel</li>
 * 	 <li>foot_acc</li>
 * 	 Note that these quantities have to be expressed in the above mentioned
 * 	 frames and orders.
 * </ul>
 */
class ReducedBodyState
{
	public:
		ReducedBodyState();
		~ReducedBodyState();

		// CoM state getter functions
		/** @brief Gets the CoM position in the world frame */
		Eigen::Vector3d getCoMPosition_W() const;

		/** @brief Gets the base orientation in the world frame */
		Eigen::Quaterniond getOrientation_W() const;

		/** @brief Gets the base RPY angles in the world frame */
		Eigen::Vector3d getRPY_W() const;

		/** @brief Gets the CoM velocity in the world frame */
		Eigen::Vector3d getCoMVelocity_W() const;

		/** @brief Gets the CoM velocity in the base frame */
		Eigen::Vector3d getCoMVelocity_B() const;

		/** @brief Gets the CoM velocity in the horizontal frame */
		Eigen::Vector3d getCoMVelocity_H() const;

		/** @brief Gets the base rotation rate in the world frame */
		Eigen::Vector3d getRotationRate_W() const;

		/** @brief Gets the base rotation rate in the base frame */
		Eigen::Vector3d getRotationRate_B() const;

		/** @brief Gets the base rotation rate in the horizontal frame */
		Eigen::Vector3d getRotationRate_H() const;

		/** @brief Gets the CoM acceleration in the world frame */
		Eigen::Vector3d getCoMAcceleration_W() const;

		/** @brief Gets the CoM acceleration in the base frame */
		Eigen::Vector3d getCoMAcceleration_B() const;

		/** @brief Gets the CoM acceleration in the horizontal frame */
		Eigen::Vector3d getCoMAcceleration_H() const;

		/** @brief Gets the base rotation acceleration in the world frame */
		Eigen::Vector3d getRotAcceleration_W() const;

		/** @brief Gets the base rotation acceleration in the base frame */
		Eigen::Vector3d getRotAcceleration_B() const;

		/** @brief Gets the base rotation acceleration in the horizontal frame */
		Eigen::Vector3d getRotAcceleration_H() const;

		// Base state setter functions
		/** @brief Sets the CoM position in the world frame */
		void setCoMPosition_W(const Eigen::Vector3d& pos_W);

		/** @brief Sets the CoM position in the base frame */
		void setCoMPosition_B(const Eigen::Vector3d& pos_B);

		/** @brief Sets the CoM position in the horizontal frame */
		void setCoMPosition_H(const Eigen::Vector3d& pos_H);

		/** @brief Sets the base orientation in the world frame */
		void setOrientation_W(const Eigen::Quaterniond& orient_W);

		/** @brief Sets the base RPY angles in the world frame */
		void setRPY_W(const Eigen::Vector3d& rpy_W);

		/** @brief Sets the CoM velocity in the world frame */
		void setCoMVelocity_W(const Eigen::Vector3d& vel_W);

		/** @brief Sets the CoM velocity in the base frame */
		void setCoMVelocity_B(const Eigen::Vector3d& vel_B);

		/** @brief Sets the CoM velocity in the horizontal frame */
		void setCoMVelocity_H(const Eigen::Vector3d& vel_H);

		/** @brief Sets the base rotation rate in the world frame */
		void setRotationRate_W(const Eigen::Vector3d& rate_W);

		/** @brief Sets the base rotation rate in the base frame */
		void setRotationRate_B(const Eigen::Vector3d& rate_B);

		/** @brief Sets the base rotation rate in the horizontal frame */
		void setRotationRate_H(const Eigen::Vector3d& rate_H);

		/** @brief Sets the CoM acceleration in the world frame */
		void setCoMAcceleration_W(const Eigen::Vector3d& acc_W);

		/** @brief Sets the CoM acceleration in the base frame */
		void setCoMAcceleration_B(const Eigen::Vector3d& acc_B);

		/** @brief Sets the CoM acceleration in the horizontal frame */
		void setCoMAcceleration_H(const Eigen::Vector3d& acc_H);

		/** @brief Sets the base rotation acceleration in the world frame */
		void setRotAcceleration_W(const Eigen::Vector3d& rotacc_W);

		/** @brief Sets the base rotation acceleration in the base frame */
		void setRotAcceleration_B(const Eigen::Vector3d& rotacc_B);

		/** @brief Sets the base rotation acceleration in the horizontal frame */
		void setRotAcceleration_H(const Eigen::Vector3d& rotacc_H);


		double time;
		Eigen::Vector3d com_pos;
		Eigen::Vector3d angular_pos;
		Eigen::Vector3d com_vel;
		Eigen::Vector3d angular_vel;
		Eigen::Vector3d com_acc;
		Eigen::Vector3d angular_acc;
		Eigen::Vector3d cop;
		rbd::BodyVector3d support_region;
		rbd::BodyVector3d foot_pos;
		rbd::BodyVector3d foot_vel;
		rbd::BodyVector3d foot_acc;


	private:
		/** @brief Frame transformations */
		math::FrameTF frame_tf_;
};

/** @brief Defines a reduced-body trajectory */
typedef std::vector<ReducedBodyState> ReducedBodyTrajectory;

} //@namespace dwl

#endif
