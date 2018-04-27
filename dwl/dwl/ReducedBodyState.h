#ifndef DWL__REDUCED_BODY_STATE__H
#define DWL__REDUCED_BODY_STATE__H

#include <Eigen/Dense>
#include <map>
#include <vector>

#include <dwl/utils/FrameTF.h>
#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

typedef Eigen::Vector3dMap::const_iterator FootIterator;

/**
 * @brief The ReducedBodyState class
 * This class describes the reduced-body state of the robot, including:
 * <ul>
 *    <li>time, in seconds</li>
 *    <li>CoM position, expressed in the world frame</li>
 *    <li>CoM velocity, expressed in the world frame</li>
 *    <li>CoM acceleration, expressed in the world frame</li>
 *    <li>CoM orientation, expressed in a world frame</li>
 *    <li>CoM angular velocity (rotation rate), expressed in the world frame</li>
 *    <li>CoM angular acceleration, expressed in the world frame</li>
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
 * B (base or CoM frame) and H (horizontal frame). Every getter/setter methods
 * have one of these three letters at the end of the signature, to clearly state
 * what is the reference frame adopted. Also convenient methods to set or get
 * RPY angles and derivatives are provided. Both Eigen and base types are
 * available for setter methods. You could also interact with the states without
 * using the getter and setter functions. Note that the states are:
 * <ul>
 *   <li>time in seconds</li>
 * 	 <li>com_pos [pos_x, pos_y, pos_z] expressed in the world frame</li>
 * 	 <li>com_vel [vel_x, vel_y, vel_z] expressed in the world frame</li>
 * 	 <li>com_acc [acc_x, acc_y, acc_z] expressed in the world frame</li>
 * 	 <li>angular_pos [roll, pitch, yaw] expressed in the world frame</li>
 * 	 <li>angular_vel [rate_x, rate_y, rate_z] expressed in the world frame</li>
 * 	 <li>angular_acc [rotacc_x, rotacc_y, rotacc_y] expressed in the world frame</li>
 * 	 <li>cop [pos_x, pos_y, pos_z] expressed in the world frame</li>
 * 	 <li>support_region</li>
 * 	 <li>foot_pos [x_0, ..., x_P] expressed in the CoM frame</li>
 * 	 <li>foot_vel [xd_0, ..., xd_P] expressed in the CoM frame</li>
 * 	 <li>foot_acc [xdd_0, ..., xdd_P] expressed in the CoM frame</li>
 * 	 where N and P are the number of DoF and contacts, respectively. Note that
 * 	 these quantities have to be expressed in the above mentioned frames and
 * 	 orders.
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 * </ul>
 */
class ReducedBodyState
{
	public:
		/** @brief Constructor function */
		ReducedBodyState();

		/** @brief Destructor function */
		~ReducedBodyState();

		// Time getter function
		/** @brief Gets the time value
		 * @return The time value
		 */
		const double& getTime() const;

		// CoM state getter functions
		/** @brief Gets the CoM position expressed in the world frame
		 * @return The CoM position
		 */
		const Eigen::Vector3d& getCoMPosition() const;

		/** @brief Gets the quaternion of the CoM frame expressed in the world frame
		 * @return The quaternion of the CoM frame
		 */
		Eigen::Quaterniond getOrientation() const;

		/** @brief Gets the RPY angles of the CoM frame expressed in the world frame
		 * @return The RPY angles of the CoM frame
		 */
		const Eigen::Vector3d& getRPY() const;

		/** @brief Gets the CoM velocity expressed in the world frame
		 * @return The CoM velocity expressed in the world frame
		 */
		const Eigen::Vector3d& getCoMVelocity_W() const;

		/** @brief Gets the CoM velocity expressed in the base frame
		 * @return The CoM velocity expressed in the base frame
		 */
		Eigen::Vector3d getCoMVelocity_B() const;

		/** @brief Gets the CoM velocity expressed in the horizontal frame
		 * @return The CoM velocity expressed in the horizontal frame
		 */
		Eigen::Vector3d getCoMVelocity_H() const;

		/** @brief Gets the angular velocity of the CoM frame expressed in the world frame
		 * @return The angular velocity of the CoM frame
		 */
		const Eigen::Vector3d& getAngularVelocity_W() const;

		/** @brief Gets the CoM angular velocity of the CoM frame expressed in the base frame
		 * @return The angular velocity of the CoM frame
		 */
		Eigen::Vector3d getAngularVelocity_B() const;

		/** @brief Gets the CoM angular velocity expressed in the horizontal frame
		 * @return The angular velocity of the CoM frame
		 */
		Eigen::Vector3d getAngularVelocity_H() const;

		/** @brief Gets the RPY velocity of the CoM frame expressed in the world frame
		 * @return The RPY velocity of the CoM frame
		 */
		Eigen::Vector3d getRPYVelocity_W() const;

		/** @brief Gets the CoM acceleration expressed in the world frame
		 * @return The CoM acceleration
		 */
		const Eigen::Vector3d& getCoMAcceleration_W() const;

		/** @brief Gets the CoM acceleration expressed in the base frame
		 * @return The CoM acceleration
		 */
		Eigen::Vector3d getCoMAcceleration_B() const;

		/** @brief Gets the CoM acceleration expressed in the horizontal frame
		 * @return The CoM acceleration
		 */
		Eigen::Vector3d getCoMAcceleration_H() const;

		/** @brief Gets the angular acceleration of the CoM frame expressed in the world frame
		 * @return The angular acceleration of the CoM frame
		 */
		const Eigen::Vector3d& getAngularAcceleration_W() const;

		/** @brief Gets the angular acceleration of the CoM frame expressed in the base frame
		 * @return The angular acceleration of the CoM frame
		 */
		Eigen::Vector3d getAngularAcceleration_B() const;

		/** @brief Gets the angular acceleration of the CoM frame expressed in the horizontal frame
		 * @return The angular acceleration of the CoM frame
		 */
		Eigen::Vector3d getAngularAcceleration_H() const;

		/** @brief Gets the RPY acceleration of the CoM frame expressed in the world frame
		 * @return The RPY acceleration of the CoM frame
		 */
		Eigen::Vector3d getRPYAcceleration_W() const;


		// CoP state getter functions
		/** @brief Gets the CoP position expressed in the world frame
		 * @return The CoP position
		 */
		const Eigen::Vector3d& getCoPPosition_W() const;


		// Foot state getter functions
		/** @brief Gets the foot position expressed the world frame
		 * @param[in] pos_it The foot position iterator
		 * @return The foot position expressed in the world frame
		 */
		Eigen::Vector3d getFootPosition_W(FootIterator pos_it) const;

		/** @brief Gets the foot position expressed the world frame
		 * @param[in] name The foot name
		 * @return The foot position expressed in the world frame
		 */
		Eigen::Vector3d getFootPosition_W(const std::string& name) const;

		/** @brief Gets all foot positions expressed the world frame
		 * @return All foot positions expressed in the world frame
		 */
		Eigen::Vector3dMap getFootPosition_W() const;

		/** @brief Gets the foot position expressed the CoM frame
		 * @param[in] pos_it The foot position iterator
		 * @return The foot position expressed in the CoM frame
		 */
		const Eigen::Vector3d& getFootPosition_B(FootIterator pos_it) const;

		/** @brief Gets the foot position expressed the CoM frame
		 * @param[in] name The foot name
		 * @return The foot position expressed in the CoM frame
		 */
		const Eigen::Vector3d& getFootPosition_B(const std::string& name) const;

		/** @brief Gets all foot positions expressed the world frame
		 * @return All foot positions expressed in the world frame
		 */
		const Eigen::Vector3dMap& getFootPosition_B() const;

		/** @brief Gets the foot position expressed the horizontal frame
		 * @param[in] pos_it The foot position iterator
		 * @return The foot position expressed in the horizontal frame
		 */
		Eigen::Vector3d getFootPosition_H(FootIterator pos_it) const;

		/** @brief Gets the foot position expressed the horizontal frame
		 * @param[in] name The foot name
		 * @return The foot position expressed in the horizontal frame
		 */
		Eigen::Vector3d getFootPosition_H(const std::string& name) const;

		/** @brief Gets all foot positions expressed the horizontal frame
		 * @return All foot positions expressed in the horizontal frame
		 */
		Eigen::Vector3dMap getFootPosition_H() const;

		/** @brief Gets the foot velocity expressed the world frame
		 * @param[in] vel_it The foot velocity iterator
		 * @return The foot velocity expressed in the world frame
		 */
		Eigen::Vector3d getFootVelocity_W(FootIterator vel_it) const;

		/** @brief Gets the foot velocity expressed the world frame
		 * @param[in] name The foot name
		 * @return The foot velocity expressed in the world frame
		 */
		Eigen::Vector3d getFootVelocity_W(const std::string& name) const;

		/** @brief Gets all foot velocities expressed the world frame
		 * @return All foot velocities expressed in the world frame
		 */
		Eigen::Vector3dMap getFootVelocity_W() const;

		/** @brief Gets the foot velocity expressed the base frame
		 * @param[in] vel_it The foot velocity iterator
		 * @return The foot velocity expressed in the base frame
		 */
		const Eigen::Vector3d& getFootVelocity_B(FootIterator vel_it) const;

		/** @brief Gets the foot velocity expressed the base frame
		 * @param[in] name The foot name
		 * @return The foot velocity expressed in the base frame
		 */
		const Eigen::Vector3d& getFootVelocity_B(const std::string& name) const;

		/** @brief Gets all foot velocities expressed the base frame
		 * @return All foot velocities expressed in the base frame
		 */
		const Eigen::Vector3dMap& getFootVelocity_B() const;

		/** @brief Gets the foot velocity expressed the horizontal frame
		 * @param[in] vel_it The foot velocity iterator
		 * @return The foot velocity expressed in the horizontal frame
		 */
		Eigen::Vector3d getFootVelocity_H(FootIterator vel_it) const;

		/** @brief Gets the foot velocity expressed the horizontal frame
		 * @param[in] name The foot name
		 * @return The foot velocity expressed in the horizontal frame
		 */
		Eigen::Vector3d getFootVelocity_H(const std::string& name) const;

		/** @brief Gets all foot velocities expressed the horizontal frame
		 * @return All foot velocities expressed in the horizontal frame
		 */
		Eigen::Vector3dMap getFootVelocity_H() const;

		/** @brief Gets the foot acceleration expressed the world frame
		 * @param[in] acc_it The foot acceleration iterator
		 * @return The foot acceleration expressed in the world frame
		 */
		Eigen::Vector3d getFootAcceleration_W(FootIterator acc_it) const;

		/** @brief Gets the foot acceleration expressed the world frame
		 * @param[in] name The foot name
		 * @return The foot acceleration expressed in the world frame
		 */
		Eigen::Vector3d getFootAcceleration_W(const std::string& name) const;

		/** @brief Gets all foot accelerations expressed the world frame
		 * @return All foot accelerations expressed in the world frame
		 */
		Eigen::Vector3dMap getFootAcceleration_W() const;

		/** @brief Gets the foot acceleration expressed the base frame
		 * @param[in] acc_it The foot acceleration iterator
		 * @return The foot acceleration expressed in the base frame
		 */
		const Eigen::Vector3d& getFootAcceleration_B(FootIterator acc_it) const;

		/** @brief Gets the foot acceleration expressed the base frame
		 * @param[in] name The foot name
		 * @return The foot acceleration expressed in the base frame
		 */
		const Eigen::Vector3d& getFootAcceleration_B(const std::string& name) const;

		/** @brief Gets all foot accelerations expressed the base frame
		 * @return All foot accelerations expressed in the base frame
		 */
		const Eigen::Vector3dMap& getFootAcceleration_B() const;

		/** @brief Gets the foot acceleration expressed the horizontal frame
		 * @param[in] acc_it The foot acceleration iterator
		 * @return The foot acceleration expressed in the horizontal frame
		 */
		Eigen::Vector3d getFootAcceleration_H(FootIterator acc_it) const;

		/** @brief Gets the foot acceleration expressed the horizontal frame
		 * @param[in] name The foot name
		 * @return The foot acceleration expressed in the horizontal frame
		 */
		Eigen::Vector3d getFootAcceleration_H(const std::string& name) const;

		/** @brief Gets all foot accelerations expressed the horizontal frame
		 * @return All foot accelerations expressed in the horizontal frame
		 */
		Eigen::Vector3dMap getFootAcceleration_H() const;


		// Time setter function
		/** @brief Sets the time value
		 * param[in] time Time value
		 */
		void setTime(const double& time);

		// CoM state setter functions
		/** @brief Sets the CoM position expressed in the world frame
		 * @param[in] pos CoM position
		 */
		void setCoMPosition(const Eigen::Vector3d& pos);

		/** @brief Sets the quaternion of the CoM frame
		 * @param[in] q Quaternion of the CoM frame
		 */
		void setOrientation(const Eigen::Quaterniond& q);

		/** @brief Sets the RPY angles of the CoM frame
		 * @param[in] rpy RPY angle of the CoM frame
		 */
		void setRPY(const Eigen::Vector3d& rpy);

		/** @brief Sets the CoM velocity expressed in the world frame
		 * @param[in] vel_W CoM velocity expressed in the world frame
		 */
		void setCoMVelocity_W(const Eigen::Vector3d& vel_W);

		/** @brief Sets the CoM velocity expressed in the base frame
		 * @param[in] vel_B CoM velocity expressed in the base frame
		 */
		void setCoMVelocity_B(const Eigen::Vector3d& vel_B);

		/** @brief Sets the CoM velocity expressed in the horizontal frame
		 * @param[in] vel_H CoM velocity expressed in the horizontal frame
		 */
		void setCoMVelocity_H(const Eigen::Vector3d& vel_H);

		/** @brief Sets the angular velocity expressed in the world frame
		 * @param[in] rate_W Angular velocity of CoM frame
		 */
		void setAngularVelocity_W(const Eigen::Vector3d& rate_W);

		/** @brief Sets the angular velocity expressed in the base frame
		 * @param[in] rate_B Angular velocity of CoM frame
		 */
		void setAngularVelocity_B(const Eigen::Vector3d& rate_B);

		/** @brief Sets the angular velocity expressed in the horizontal frame
		 * @param[in] rate_H Angular velocity of CoM frame
		 */
		void setAngularVelocity_H(const Eigen::Vector3d& rate_H);

		/** @brief Sets the RPY velocity of the CoM frame expressed in the world frame
		 * @param[in] rpy_rate Angular velocity of CoM frame
		 */
		void setRPYVelocity_W(const Eigen::Vector3d& rpy_rate);

		/** @brief Sets the CoM acceleration expressed in the world frame
		 * @param[in] acc_W CoM acceleration expressed in the world frame
		 */
		void setCoMAcceleration_W(const Eigen::Vector3d& acc_W);

		/** @brief Sets the CoM acceleration expressed in the base frame
		 * @param[in] acc_B CoM acceleration expressed in the base frame
		 */
		void setCoMAcceleration_B(const Eigen::Vector3d& acc_B);

		/** @brief Sets the CoM acceleration expressed in the horizontal frame
		 * @param[in] acc_H CoM acceleration expressed in the horizontal frame
		 */
		void setCoMAcceleration_H(const Eigen::Vector3d& acc_H);

		/** @brief Sets the angular acceleration of the CoM frame expressed in the world frame
		 * @param[in] rotacc_W Angular acceleration of the CoM frame
		 */
		void setAngularAcceleration_W(const Eigen::Vector3d& rotacc_W);

		/** @brief Sets the angular acceleration of the CoM frame expressed in the base frame
		 * @param[in] rotacc_B Angular acceleration of the CoM frame
		 */
		void setAngularAcceleration_B(const Eigen::Vector3d& rotacc_B);

		/** @brief Sets the angular acceleration of the CoM frame expressed in the horizontal frame
		 * @param[in] rotacc_H Angular acceleration of the CoM frame
		 */
		void setAngularAcceleration_H(const Eigen::Vector3d& rotacc_H);

		/** @brief Sets the RPY acceleration of the CoM frame expressed in the world frame
		 * @param[in] rpy_rate RPY acceleration of the CoM frame
		 */
		void setRPYAcceleration_W(const Eigen::Vector3d& rpy_rate);


		// CoP state setter functions
		/** @brief Sets the CoP position expressed in the world frame
		 * @param[in] cop_W CoP position expressed in the world frame
		 */
		void setCoPPosition_W(const Eigen::Vector3d& cop_W);


		// Foot state setter functions
		/** @brief Sets the foot position expressed in the world frame
		 * @param[in] pos_it Foot position iterator
		 */
		void setFootPosition_W(FootIterator pos_it);

		/** @brief Sets the foot position expressed in the world frame
		 * @param[in] name Foot name
		 * @param[in] pos_W Foot position iterator
		 */
		void setFootPosition_W(const std::string& name,
							   const Eigen::Vector3d& pos_W);

		/** @brief Sets all foot positions expressed in the world frame
		 * @param[in] pos_W All foot positions iterator
		 */
		void setFootPosition_W(const Eigen::Vector3dMap& pos_W);

		/** @brief Sets the foot position expressed in the CoM frame
		 * @param[in] pos_it Foot position iterator
		 */
		void setFootPosition_B(FootIterator pos_it);

		/** @brief Sets the foot position expressed in the CoM frame
		 * @param[in] name Foot name
		 * @param[in] pos_B Foot position iterator
		 */
		void setFootPosition_B(const std::string& name,
							   const Eigen::Vector3d& pos_B);

		/** @brief Sets all foot positions expressed in the CoM frame
		 * @param[in] pos_B All foot positions iterator
		 */
		void setFootPosition_B(const Eigen::Vector3dMap& pos_B);

		/** @brief Sets the foot position expressed in the horizontal frame
		 * @param[in] pos_it Foot position iterator
		 */
		void setFootPosition_H(FootIterator pos_it);

		/** @brief Sets the foot position expressed in the horizontal frame
		 * @param[in] name Foot name
		 * @param[in] pos_H Foot position iterator
		 */
		void setFootPosition_H(const std::string& name,
							   const Eigen::Vector3d& pos_H);

		/** @brief Sets all foot positions expressed in the horizontal frame
		 * @param[in] pos_H All foot positions iterator
		 */
		void setFootPosition_H(const Eigen::Vector3dMap& pos_H);

		/** @brief Sets the foot velocity expressed in the world frame
		 * @param[in] vel_it Foot velocity iterator
		 */
		void setFootVelocity_W(FootIterator it);

		/** @brief Sets the foot velocity expressed in the world frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot velocity iterator
		 */
		void setFootVelocity_W(const std::string& name,
							   const Eigen::Vector3d& vel_W);

		/** @brief Sets all foot velocities expressed in the world frame
		 * @param[in] vel_W All foot velocities iterator
		 */
		void setFootVelocity_W(const Eigen::Vector3dMap& vel_W);

		/** @brief Sets the foot velocity expressed in the base frame
		 * @param[in] vel_it Foot velocity iterator
		 */
		void setFootVelocity_B(FootIterator vel_it);

		/** @brief Sets the foot velocity expressed in the base frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot velocity iterator
		 */
		void setFootVelocity_B(const std::string& name,
							   const Eigen::Vector3d& vel_B);

		/** @brief Sets all foot velocities expressed in the base frame
		 * @param[in] vel_B All foot velocities iterator
		 */
		void setFootVelocity_B(const Eigen::Vector3dMap& vel_B);

		/** @brief Sets the foot velocity expressed in the horizontal frame
		 * @param[in] vel_it Foot velocity iterator
		 */
		void setFootVelocity_H(FootIterator vel_it);

		/** @brief Sets the foot velocity expressed in the horizontal frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot velocity iterator
		 */
		void setFootVelocity_H(const std::string& name,
							   const Eigen::Vector3d& vel_H);

		/** @brief Sets all foot velocities expressed in the horizontal frame
		 * @param[in] vel_H All foot velocities iterator
		 */
		void setFootVelocity_H(const Eigen::Vector3dMap& vel_H);

		/** @brief Sets the foot acceleration expressed in the world frame
		 * @param[in] acc_it Foot acceleration iterator
		 */
		void setFootAcceleration_W(FootIterator acc_it);

		/** @brief Sets the foot acceleration expressed in the world frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot acceleration iterator
		 */
		void setFootAcceleration_W(const std::string& name,
								   const Eigen::Vector3d& acc_W);

		/** @brief Sets all foot accelerations expressed in the world frame
		 * @param[in] acc_W All foot accelerations iterator
		 */
		void setFootAcceleration_W(const Eigen::Vector3dMap& acc_W);

		/** @brief Sets the foot acceleration expressed in the base frame
		 * @param[in] acc_it Foot acceleration iterator
		 */
		void setFootAcceleration_B(FootIterator acc_it);

		/** @brief Sets the foot acceleration expressed in the base frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot acceleration iterator
		 */
		void setFootAcceleration_B(const std::string& name,
								   const Eigen::Vector3d& acc_B);

		/** @brief Sets all foot accelerations expressed in the base frame
		 * @param[in] acc_B All foot accelerations iterator
		 */
		void setFootAcceleration_B(const Eigen::Vector3dMap& acc_B);

		/** @brief Sets the foot acceleration expressed in the horizontal frame
		 * @param[in] acc_it Foot acceleration iterator
		 */
		void setFootAcceleration_H(FootIterator acc_it);

		/** @brief Sets the foot acceleration expressed in the horizontal frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot acceleration iterator
		 */
		void setFootAcceleration_H(const std::string& name,
								   const Eigen::Vector3d& acc_H);

		/** @brief Sets all foot accelerations expressed in the horizontal frame
		 * @param[in] acc_H All foot accelerations iterator
		 */
		void setFootAcceleration_H(const Eigen::Vector3dMap& acc_H);


		double time;
		Eigen::Vector3d com_pos;
		Eigen::Vector3d angular_pos;
		Eigen::Vector3d com_vel;
		Eigen::Vector3d angular_vel;
		Eigen::Vector3d com_acc;
		Eigen::Vector3d angular_acc;
		Eigen::Vector3d cop;
		Eigen::Vector3dMap support_region;
		Eigen::Vector3dMap foot_pos;
		Eigen::Vector3dMap foot_vel;
		Eigen::Vector3dMap foot_acc;


	private:
		/** @brief Frame transformations */
		math::FrameTF frame_tf_;
};

/** @brief Defines a reduced-body trajectory */
typedef std::vector<ReducedBodyState> ReducedBodyTrajectory;

} //@namespace dwl

#endif
