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
	typedef rbd::BodyVector3d::const_iterator FootIterator;

	public:
		/** @brief Constructor function */
		ReducedBodyState();

		/** @brief Destructor function */
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

		/** @brief Gets the base angular velocity in the world frame */
		Eigen::Vector3d getAngularVelocity_W() const;

		/** @brief Gets the base angular velocity in the base frame */
		Eigen::Vector3d getAngularVelocity_B() const;

		/** @brief Gets the base angular velocity in the horizontal frame */
		Eigen::Vector3d getAngularVelocity_H() const;

		/** @brief Gets the CoM acceleration in the world frame */
		Eigen::Vector3d getCoMAcceleration_W() const;

		/** @brief Gets the CoM acceleration in the base frame */
		Eigen::Vector3d getCoMAcceleration_B() const;

		/** @brief Gets the CoM acceleration in the horizontal frame */
		Eigen::Vector3d getCoMAcceleration_H() const;

		/** @brief Gets the base angular acceleration in the world frame */
		Eigen::Vector3d getAngularAcceleration_W() const;

		/** @brief Gets the base angular acceleration in the base frame */
		Eigen::Vector3d getAngularAcceleration_B() const;

		/** @brief Gets the base angular acceleration in the horizontal frame */
		Eigen::Vector3d getAngularAcceleration_H() const;

		// Foot state getter functions
		/** @brief Gets the foot position expressed in the world frame */
		Eigen::Vector3d getFootPosition_W(FootIterator it) const;
		Eigen::Vector3d getFootPosition_W(std::string name) const;
		rbd::BodyVector3d getFootPosition_W() const;

		/** @brief Gets the foot position expressed in the base frame */
		Eigen::Vector3d getFootPosition_B(FootIterator it) const;
		Eigen::Vector3d getFootPosition_B(std::string name) const;
		rbd::BodyVector3d getFootPosition_B() const;

		/** @brief Gets the foot position expressed in the horizontal frame */
		Eigen::Vector3d getFootPosition_H(FootIterator it) const;
		Eigen::Vector3d getFootPosition_H(std::string name) const;
		rbd::BodyVector3d getFootPosition_H() const;

		/** @brief Gets the foot velocity expressed in the world frame */
		Eigen::Vector3d getFootVelocity_W(FootIterator it) const;
		Eigen::Vector3d getFootVelocity_W(std::string name) const;
		rbd::BodyVector3d getFootVelocity_W() const;

		/** @brief Gets the foot velocity expressed in the base frame */
		Eigen::Vector3d getFootVelocity_B(FootIterator it) const;
		Eigen::Vector3d getFootVelocity_B(std::string name) const;
		rbd::BodyVector3d getFootVelocity_B() const;

		/** @brief Gets the foot velocity expressed in the horizontal frame */
		Eigen::Vector3d getFootVelocity_H(FootIterator it) const;
		Eigen::Vector3d getFootVelocity_H(std::string name) const;
		rbd::BodyVector3d getFootVelocity_H() const;

		/** @brief Gets the foot acceleration expressed in the world frame */
		Eigen::Vector3d getFootAcceleration_W(FootIterator it) const;
		Eigen::Vector3d getFootAcceleration_W(std::string name) const;
		rbd::BodyVector3d getFootAcceleration_W() const;

		/** @brief Gets the foot acceleration expressed in the base frame */
		Eigen::Vector3d getFootAcceleration_B(FootIterator it) const;
		Eigen::Vector3d getFootAcceleration_B(std::string name) const;
		rbd::BodyVector3d getFootAcceleration_B() const;

		/** @brief Gets the foot acceleration expressed in the horizontal frame */
		Eigen::Vector3d getFootAcceleration_H(FootIterator it) const;
		Eigen::Vector3d getFootAcceleration_H(std::string name) const;
		rbd::BodyVector3d getFootAcceleration_H() const;


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

		/** @brief Sets the base angular velocity in the world frame */
		void setAngularVelocity_W(const Eigen::Vector3d& rate_W);

		/** @brief Sets the base angular velocity in the base frame */
		void setAngularVelocity_B(const Eigen::Vector3d& rate_B);

		/** @brief Sets the base angular velocity in the horizontal frame */
		void setAngularVelocity_H(const Eigen::Vector3d& rate_H);

		/** @brief Sets the CoM acceleration in the world frame */
		void setCoMAcceleration_W(const Eigen::Vector3d& acc_W);

		/** @brief Sets the CoM acceleration in the base frame */
		void setCoMAcceleration_B(const Eigen::Vector3d& acc_B);

		/** @brief Sets the CoM acceleration in the horizontal frame */
		void setCoMAcceleration_H(const Eigen::Vector3d& acc_H);

		/** @brief Sets the base angular acceleration in the world frame */
		void setAngularAcceleration_W(const Eigen::Vector3d& rotacc_W);

		/** @brief Sets the base angular acceleration in the base frame */
		void setAngularAcceleration_B(const Eigen::Vector3d& rotacc_B);

		/** @brief Sets the base angular acceleration in the horizontal frame */
		void setAngularAcceleration_H(const Eigen::Vector3d& rotacc_H);


		// Foot state setter functions
		/** @brief Sets the foot position expressed in the world frame */
		void setFootPosition_W(FootIterator it);
		void setFootPosition_W(std::string name,
							   const Eigen::Vector3d& pos_W);
		void setFootPosition_W(const rbd::BodyVector3d& pos_W);

		/** @brief Sets the foot position expressed in the base frame */
		void setFootPosition_B(FootIterator it);
		void setFootPosition_B(std::string name,
							   const Eigen::Vector3d& pos_B);
		void setFootPosition_B(const rbd::BodyVector3d& pos_B);

		/** @brief Sets the foot position expressed in the horizontal frame */
		void setFootPosition_H(FootIterator it);
		void setFootPosition_H(std::string name,
							   const Eigen::Vector3d& pos_H);
		void setFootPosition_H(const rbd::BodyVector3d& pos_H);

		/** @brief Sets the foot velocity expressed in the world frame */
		void setFootVelocity_W(FootIterator it);
		void setFootVelocity_W(std::string name,
							   const Eigen::Vector3d& vel_W);
		void setFootVelocity_W(const rbd::BodyVector3d& vel_W);

		/** @brief Sets the foot velocity expressed in the base frame */
		void setFootVelocity_B(FootIterator it);
		void setFootVelocity_B(std::string name,
							   const Eigen::Vector3d& vel_B);
		void setFootVelocity_B(const rbd::BodyVector3d& vel_B);

		/** @brief Sets the foot velocity expressed in the horizontal frame */
		void setFootVelocity_H(FootIterator it);
		void setFootVelocity_H(std::string name,
							   const Eigen::Vector3d& vel_H);
		void setFootVelocity_H(const rbd::BodyVector3d& vel_H);

		/** @brief Sets the foot acceleration expressed in the world frame */
		void setFootAcceleration_W(FootIterator vel_it,
				   	   	   	   	   FootIterator acc_it);
		void setFootAcceleration_W(std::string name,
								   const Eigen::Vector3d& vel_W,
								   const Eigen::Vector3d& acc_W);
		void setFootAcceleration_W(const rbd::BodyVector3d& vel_W,
								   const rbd::BodyVector3d& acc_W);

		/** @brief Sets the foot acceleration expressed in the base frame */
		void setFootAcceleration_B(FootIterator it);
		void setFootAcceleration_B(std::string name,
								   const Eigen::Vector3d& acc_B);
		void setFootAcceleration_B(const rbd::BodyVector3d& acc_B);

		/** @brief Sets the foot acceleration expressed in the horizontal frame */
		void setFootAcceleration_H(FootIterator vel_it,
				   	   	   	   	   FootIterator acc_it);
		void setFootAcceleration_H(std::string name,
								   const Eigen::Vector3d& vel_H,
								   const Eigen::Vector3d& acc_H);
		void setFootAcceleration_H(const rbd::BodyVector3d& vel_H,
								   const rbd::BodyVector3d& acc_H);


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
		/**
		 * @brief Computes the relative foot velocity w.r.t. the base expressed
		 * in the world frame
		 * @param std::string Name of the foot
		 * @param const Eigen::Vector3d& Foot velocity expressed in the world frame
		 */
		Eigen::Vector3d computeRelativeFootVelocity_W(std::string name,
													  const Eigen::Vector3d& vel_W);

		/**
		 * @brief Computes the relative foot acceleration w.r.t. the base expressed
		 * in the world frame
		 * @param std::string Name of the foot
		 * @param const Eigen::Vector3d& Foot velocity expressed in the world frame
		 * @param const Eigen::Vector3d& Foot acceleration expressed in the world frame
		 */
		Eigen::Vector3d computeRelativeFootAcceleration_W(std::string name,
														  const Eigen::Vector3d& vel_W,
														  const Eigen::Vector3d& acc_W);

		/** @brief Frame transformations */
		math::FrameTF frame_tf_;
};

/** @brief Defines a reduced-body trajectory */
typedef std::vector<ReducedBodyState> ReducedBodyTrajectory;

} //@namespace dwl

#endif
