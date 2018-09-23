#ifndef DWL__REDUCED_BODY_STATE__H
#define DWL__REDUCED_BODY_STATE__H

#include <Eigen/Dense>
#include <map>
#include <vector>

#include <dwl/utils/LieGroup.h>
#include <dwl/utils/FrameTF.h>
#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

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
		typedef dwl::SE3Map::const_iterator SE3Iterator;
		typedef dwl::MotionMap::const_iterator MotionIterator;

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
		/**
		 * @brief Gets the CoM SE3 expressed in the world frame
		 * @return The CoM position
		 */
		const dwl::SE3& getCoMSE3() const;

		/**
		 * @brief Gets the base SE3 of the horizontal frame
		 * @return Base SE3 of the horizontal frame
		 */
		const dwl::SE3& getCoMSE3_H();

		/** @brief Gets the CoM velocity expressed in the world frame
		 * @return The CoM velocity expressed in the world frame
		 */
		const dwl::Motion& getCoMVelocity_W() const;

		/** @brief Gets the CoM velocity expressed in the base frame
		 * @return The CoM velocity expressed in the base frame
		 */
		const dwl::Motion& getCoMVelocity_B();

		/** @brief Gets the CoM velocity expressed in the horizontal frame
		 * @return The CoM velocity expressed in the horizontal frame
		 */
		const dwl::Motion& getCoMVelocity_H();

		/** @brief Gets the RPY velocity of the CoM frame expressed in the world frame
		 * @return The RPY velocity of the CoM frame
		 */
		const Eigen::Vector3d& getRPYVelocity_W();

		/** @brief Gets the CoM acceleration expressed in the world frame
		 * @return The CoM acceleration
		 */
		const dwl::Motion& getCoMAcceleration_W() const;

		/** @brief Gets the CoM acceleration expressed in the base frame
		 * @return The CoM acceleration
		 */
		const dwl::Motion& getCoMAcceleration_B();

		/** @brief Gets the CoM acceleration expressed in the horizontal frame
		 * @return The CoM acceleration
		 */
		const dwl::Motion& getCoMAcceleration_H();

		/** @brief Gets the RPY acceleration of the CoM frame expressed in the world frame
		 * @return The RPY acceleration of the CoM frame
		 */
		Eigen::Vector3d getRPYAcceleration_W();


		// CoP state getter functions
		/** @brief Gets the CoP position expressed in the world frame
		 * @return The CoP position
		 */
		const Eigen::Vector3d& getCoPPosition_W() const;


		// Foot state getter functions
		/** @brief Gets the foot position expressed the world frame
		 * @param[in] it The foot position iterator
		 * @return The foot position expressed in the world frame
		 */
		const dwl::SE3& getFootSE3_W(SE3Iterator it);

		/** @brief Gets the foot position expressed the world frame
		 * @param[in] name The foot name
		 * @return The foot position expressed in the world frame
		 */
		const dwl::SE3& getFootSE3_W(const std::string& name);

		/** @brief Gets all foot positions expressed the world frame
		 * @return All foot positions expressed in the world frame
		 */
		dwl::SE3Map getFootSE3_W();

		/** @brief Gets the foot position expressed the CoM frame
		 * @param[in] it The foot position iterator
		 * @return The foot position expressed in the CoM frame
		 */
		const dwl::SE3& getFootSE3_B(SE3Iterator it) const;

		/** @brief Gets the foot position expressed the CoM frame
		 * @param[in] name The foot name
		 * @return The foot position expressed in the CoM frame
		 */
		const dwl::SE3& getFootSE3_B(const std::string& name) const;

		/** @brief Gets all foot positions expressed the world frame
		 * @return All foot positions expressed in the world frame
		 */
		const dwl::SE3Map& getFootSE3_B() const;

		/** @brief Gets the foot position expressed the horizontal frame
		 * @param[in] it The foot position iterator
		 * @return The foot position expressed in the horizontal frame
		 */
		const dwl::SE3& getFootSE3_H(SE3Iterator it);

		/** @brief Gets the foot position expressed the horizontal frame
		 * @param[in] name The foot name
		 * @return The foot position expressed in the horizontal frame
		 */
		const dwl::SE3& getFootSE3_H(const std::string& name);

		/** @brief Gets all foot positions expressed the horizontal frame
		 * @return All foot positions expressed in the horizontal frame
		 */
		dwl::SE3Map getFootSE3_H();

		/** @brief Gets the foot velocity expressed the world frame
		 * @param[in] it The foot velocity iterator
		 * @return The foot velocity expressed in the world frame
		 */
		const dwl::Motion& getFootVelocity_W(MotionIterator it);

		/** @brief Gets the foot velocity expressed the world frame
		 * @param[in] name The foot name
		 * @return The foot velocity expressed in the world frame
		 */
		const dwl::Motion& getFootVelocity_W(const std::string& name);

		/** @brief Gets all foot velocities expressed the world frame
		 * @return All foot velocities expressed in the world frame
		 */
		dwl::MotionMap getFootVelocity_W();

		/** @brief Gets the foot velocity expressed the base frame
		 * @param[in] vel_it The foot velocity iterator
		 * @return The foot velocity expressed in the base frame
		 */
		const dwl::Motion& getFootVelocity_B(MotionIterator vel_it) const;

		/** @brief Gets the foot velocity expressed the base frame
		 * @param[in] name The foot name
		 * @return The foot velocity expressed in the base frame
		 */
		const dwl::Motion& getFootVelocity_B(const std::string& name) const;

		/** @brief Gets all foot velocities expressed the base frame
		 * @return All foot velocities expressed in the base frame
		 */
		const dwl::MotionMap& getFootVelocity_B() const;

		/** @brief Gets the foot velocity expressed the horizontal frame
		 * @param[in] it The foot velocity iterator
		 * @return The foot velocity expressed in the horizontal frame
		 */
		const dwl::Motion& getFootVelocity_H(MotionIterator it);

		/** @brief Gets the foot velocity expressed the horizontal frame
		 * @param[in] name The foot name
		 * @return The foot velocity expressed in the horizontal frame
		 */
		const dwl::Motion& getFootVelocity_H(const std::string& name);

		/** @brief Gets all foot velocities expressed the horizontal frame
		 * @return All foot velocities expressed in the horizontal frame
		 */
		dwl::MotionMap getFootVelocity_H();

		/** @brief Gets the foot acceleration expressed the world frame
		 * @param[in] it The foot acceleration iterator
		 * @return The foot acceleration expressed in the world frame
		 */
		const dwl::Motion& getFootAcceleration_W(MotionIterator it);

		/** @brief Gets the foot acceleration expressed the world frame
		 * @param[in] name The foot name
		 * @return The foot acceleration expressed in the world frame
		 */
		const dwl::Motion& getFootAcceleration_W(const std::string& name);

		/** @brief Gets all foot accelerations expressed the world frame
		 * @return All foot accelerations expressed in the world frame
		 */
		dwl::MotionMap getFootAcceleration_W();

		/** @brief Gets the foot acceleration expressed the base frame
		 * @param[in] it The foot acceleration iterator
		 * @return The foot acceleration expressed in the base frame
		 */
		const dwl::Motion& getFootAcceleration_B(MotionIterator it) const;

		/** @brief Gets the foot acceleration expressed the base frame
		 * @param[in] name The foot name
		 * @return The foot acceleration expressed in the base frame
		 */
		const dwl::Motion& getFootAcceleration_B(const std::string& name) const;

		/** @brief Gets all foot accelerations expressed the base frame
		 * @return All foot accelerations expressed in the base frame
		 */
		const dwl::MotionMap& getFootAcceleration_B() const;

		/** @brief Gets the foot acceleration expressed the horizontal frame
		 * @param[in] it The foot acceleration iterator
		 * @return The foot acceleration expressed in the horizontal frame
		 */
		const dwl::Motion& getFootAcceleration_H(MotionIterator it);

		/** @brief Gets the foot acceleration expressed the horizontal frame
		 * @param[in] name The foot name
		 * @return The foot acceleration expressed in the horizontal frame
		 */
		const dwl::Motion& getFootAcceleration_H(const std::string& name);

		/** @brief Gets all foot accelerations expressed the horizontal frame
		 * @return All foot accelerations expressed in the horizontal frame
		 */
		dwl::MotionMap getFootAcceleration_H();


		// Support region getter functions
		/** @brief Gets all foot in contacts and them positions
		 * @return All foot in contacts and them positions
		 */
		const dwl::SE3Map& getSupportRegion() const;


		// Time setter function
		/** @brief Sets the time value
		 * param[in] time Time value
		 */
		void setTime(const double& time);

		// CoM state setter functions
		/** @brief Sets the CoM position expressed in the world frame
		 * @param[in] pos CoM SE3
		 */
		void setCoMSE3(const dwl::SE3& pos);

		/** @brief Sets the CoM velocity expressed in the world frame
		 * @param[in] vel_W CoM velocity expressed in the world frame
		 */
		void setCoMVelocity_W(const dwl::Motion& vel_W);

		/** @brief Sets the CoM velocity expressed in the base frame
		 * @param[in] vel_B CoM velocity expressed in the base frame
		 */
		void setCoMVelocity_B(const dwl::Motion& vel_B);

		/** @brief Sets the CoM velocity expressed in the horizontal frame
		 * @param[in] vel_H CoM velocity expressed in the horizontal frame
		 */
		void setCoMVelocity_H(const dwl::Motion& vel_H);

		/** @brief Sets the RPY velocity of the CoM frame expressed in the world frame
		 * @param[in] rpy_rate Angular velocity of CoM frame
		 */
		void setRPYVelocity_W(const Eigen::Vector3d& rpy_rate);

		/** @brief Sets the CoM acceleration expressed in the world frame
		 * @param[in] acc_W CoM acceleration expressed in the world frame
		 */
		void setCoMAcceleration_W(const dwl::Motion& acc_W);

		/** @brief Sets the CoM acceleration expressed in the base frame
		 * @param[in] acc_B CoM acceleration expressed in the base frame
		 */
		void setCoMAcceleration_B(const dwl::Motion& acc_B);

		/** @brief Sets the CoM acceleration expressed in the horizontal frame
		 * @param[in] acc_H CoM acceleration expressed in the horizontal frame
		 */
		void setCoMAcceleration_H(const dwl::Motion& acc_H);

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
		void setFootSE3_W(SE3Iterator pos_it);

		/** @brief Sets the foot position expressed in the world frame
		 * @param[in] name Foot name
		 * @param[in] pos_W Foot position iterator
		 */
		void setFootSE3_W(const std::string& name,
						  const dwl::SE3& pos_W);

		/** @brief Sets all foot positions expressed in the world frame
		 * @param[in] pos_W All foot positions iterator
		 */
		void setFootSE3_W(const dwl::SE3Map& pos_W);

		/** @brief Sets the foot position expressed in the CoM frame
		 * @param[in] pos_it Foot position iterator
		 */
		void setFootSE3_B(SE3Iterator pos_it);

		/** @brief Sets the foot position expressed in the CoM frame
		 * @param[in] name Foot name
		 * @param[in] pos_B Foot position iterator
		 */
		void setFootSE3_B(const std::string& name,
						  const dwl::SE3& pos_B);

		/** @brief Sets all foot positions expressed in the CoM frame
		 * @param[in] pos_B All foot positions iterator
		 */
		void setFootSE3_B(const dwl::SE3Map& pos_B);

		/** @brief Sets the foot position expressed in the horizontal frame
		 * @param[in] pos_it Foot position iterator
		 */
		void setFootSE3_H(SE3Iterator pos_it);

		/** @brief Sets the foot position expressed in the horizontal frame
		 * @param[in] name Foot name
		 * @param[in] pos_H Foot position iterator
		 */
		void setFootSE3_H(const std::string& name,
						  const dwl::SE3& pos_H);

		/** @brief Sets all foot positions expressed in the horizontal frame
		 * @param[in] pos_H All foot positions iterator
		 */
		void setFootSE3_H(const dwl::SE3Map& pos_H);

		/** @brief Sets the foot velocity expressed in the world frame
		 * @param[in] vel_it Foot velocity iterator
		 */
		void setFootVelocity_W(MotionIterator it);

		/** @brief Sets the foot velocity expressed in the world frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot velocity iterator
		 */
		void setFootVelocity_W(const std::string& name,
							   const dwl::Motion& vel_W);

		/** @brief Sets all foot velocities expressed in the world frame
		 * @param[in] vel_W All foot velocities iterator
		 */
		void setFootVelocity_W(const dwl::MotionMap& vel_W);

		/** @brief Sets the foot velocity expressed in the base frame
		 * @param[in] vel_it Foot velocity iterator
		 */
		void setFootVelocity_B(MotionIterator vel_it);

		/** @brief Sets the foot velocity expressed in the base frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot velocity iterator
		 */
		void setFootVelocity_B(const std::string& name,
							   const dwl::Motion& vel_B);

		/** @brief Sets all foot velocities expressed in the base frame
		 * @param[in] vel_B All foot velocities iterator
		 */
		void setFootVelocity_B(const dwl::MotionMap& vel_B);

		/** @brief Sets the foot velocity expressed in the horizontal frame
		 * @param[in] vel_it Foot velocity iterator
		 */
		void setFootVelocity_H(MotionIterator vel_it);

		/** @brief Sets the foot velocity expressed in the horizontal frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot velocity iterator
		 */
		void setFootVelocity_H(const std::string& name,
							   const dwl::Motion& vel_H);

		/** @brief Sets all foot velocities expressed in the horizontal frame
		 * @param[in] vel_H All foot velocities iterator
		 */
		void setFootVelocity_H(const dwl::MotionMap& vel_H);

		/** @brief Sets the foot acceleration expressed in the world frame
		 * @param[in] acc_it Foot acceleration iterator
		 */
		void setFootAcceleration_W(MotionIterator acc_it);

		/** @brief Sets the foot acceleration expressed in the world frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot acceleration iterator
		 */
		void setFootAcceleration_W(const std::string& name,
								   const dwl::Motion& acc_W);

		/** @brief Sets all foot accelerations expressed in the world frame
		 * @param[in] acc_W All foot accelerations iterator
		 */
		void setFootAcceleration_W(const dwl::MotionMap& acc_W);

		/** @brief Sets the foot acceleration expressed in the base frame
		 * @param[in] acc_it Foot acceleration iterator
		 */
		void setFootAcceleration_B(MotionIterator acc_it);

		/** @brief Sets the foot acceleration expressed in the base frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot acceleration iterator
		 */
		void setFootAcceleration_B(const std::string& name,
								   const dwl::Motion& acc_B);

		/** @brief Sets all foot accelerations expressed in the base frame
		 * @param[in] acc_B All foot accelerations iterator
		 */
		void setFootAcceleration_B(const dwl::MotionMap& acc_B);

		/** @brief Sets the foot acceleration expressed in the horizontal frame
		 * @param[in] acc_it Foot acceleration iterator
		 */
		void setFootAcceleration_H(MotionIterator acc_it);

		/** @brief Sets the foot acceleration expressed in the horizontal frame
		 * @param[in] name Foot name
		 * @param[in] vel_W Foot acceleration iterator
		 */
		void setFootAcceleration_H(const std::string& name,
								   const dwl::Motion& acc_H);

		/** @brief Sets all foot accelerations expressed in the horizontal frame
		 * @param[in] acc_H All foot accelerations iterator
		 */
		void setFootAcceleration_H(const dwl::MotionMap& acc_H);

		// Support region setter functions
		/** @brief Sets all foot in contacts and them positions
		 * @return All foot in contacts and them positions
		 */
		void setSupportRegion(const dwl::SE3Map& support);


		double time;
		dwl::SE3 com_pos;
		dwl::Motion com_vel;
		dwl::Motion com_acc;
		Eigen::Vector3d cop;
		dwl::SE3Map support_region;
		dwl::SE3Map foot_pos;
		dwl::MotionMap foot_vel;
		dwl::MotionMap foot_acc;


	private:
		/** @brief Frame transformations */
		math::FrameTF frame_tf_;

		/** @brief Internal data to avoid dynamic memory allocation **/
		dwl::SE3 se3_;
		dwl::Motion motion_;
		Eigen::Vector3d vec3_;

		/** @brief Null vectors for missed contact states */
		dwl::SE3 null_se3_;
		dwl::Motion null_motion_;
		dwl::Force null_force_;
};

/** @brief Defines a reduced-body trajectory */
typedef std::vector<ReducedBodyState> ReducedBodyTrajectory;

} //@namespace dwl

#endif
