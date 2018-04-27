#ifndef DWL__WHOLE_BODY_STATE__H
#define DWL__WHOLE_BODY_STATE__H

#include <map>
#include <vector>

#include <dwl/utils/FrameTF.h>
#include <dwl/utils/EigenExtra.h>
#include <dwl/utils/RigidBodyDynamics.h>


#define NO_WRENCH Eigen::Vector6d::Zero()
#define MAX_WRENCH 2e19 * Eigen::Vector6d::Ones()
#define INACTIVE_CONTACT NO_WRENCH
#define ACTIVE_CONTACT MAX_WRENCH


namespace dwl
{

/**
 * @brief The WholeBodyState class
 * This class describes the whole-body state of the robot, including:
 * <ul>
 *    <li>time, in seconds</li>
 *    <li>Base position, expressed in the world frame</li>
 *    <li>Base orientation, expressed in a world frame</li>
 *    <li>Base velocity, expressed in the world frame</li>
 *    <li>Base angular velocity (rotation rate), expressed in the world frame</li><li>
 *    <li>Base acceleration, expressed in the world frame</li>
 *    <li>Base angular acceleration, expressed in the world frame</li>
 *    <li>Joint positions, expressed with the urdf order</li>
 *    <li>Joint velocities, expressed with the urdf order</li>
 *    <li>Joint accelerations, expressed with the urdf order</li>
 *    <li>Joint efforts, expressed with the urdf order</li>
 *    <li>Contact positions, expressed in the base frame</li>
 *    <li>Contact velocities, expressed in the base frame</li>
 *    <li>Contact accelerations, expressed in the base frame</li>
 *    <li>Contact efforts, expressed in the base frame</li>
 * </ul>
 *
 * The class provides getter and setter routines for all these states.
 * When useful, it provides alternatives to set or get the desired physical
 * quantity in one of the following frames of reference: W (fixed-world frame),
 * B (base frame) and H (horizontal frame). Every getter/setter methods have one
 * of these three letters at the end of the signature, to clearly state what is
 * the adopted reference frame. Also convenient methods to set or get RPY angles
 * and derivatives are provided. Both Eigen and base types are available for
 * setter methods. You could also interact with the states without using the
 * getter and setter routines. Note that the states are:
 * <ul>
 *   <li>time in seconds</li>
 * 	 <li>base_pos [qx, qy, qz, qw, x, y, z] expressed in the world frame </li>
 * 	 <li>base_vel [rate_x, rate_y, rate_z, x, y, z] expressed in the world frame </li>
 * 	 <li>base_acc [rotacc_x, rotacc_y, rotacc_y, x, y, z] expressed in the world frame </li>
 * 	 <li>joint_pos [q_0, ..., q_N]</li>
 * 	 <li>joint_vel [qd_0, ..., qd_N]</li>
 * 	 <li>joint_acc [qdd_0, ..., qdd_N]</li>
 * 	 <li>joint_eff [tau_0, ..., tau_N]</li>
 * 	 <li>contact_pos [x_0, ..., x_P] expressed in the base frame </li>
 * 	 <li>contact_vel [xd_0, ..., xd_P] expressed in the base frame </li>
 * 	 <li>contact_acc [xdd_0, ..., xdd_P] expressed in the base frame </li>
 * 	 <li>contact_eff [f_0, ..., f_P] expressed in the base frame </li>
 * 	 where N and P are the number of DoF and contacts, respectively. Note that
 * 	 these quantities have to be expressed in the above mentioned frames and
 * 	 orders. The number of contact states are described at runtime by using its
 * 	 name.
 * </ul>
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 */
class WholeBodyState
{
	public:
		typedef Eigen::VectorXdMap::const_iterator ContactIterator;

		/** @brief Constructor function */
		WholeBodyState(unsigned int num_joints = 0);

		/** @brief Destructor function */
		~WholeBodyState();

		// Time getter function
		/** @brief Gets the time value
		 * @return The time value
		 */
		const double& getTime() const;

		// Base state getter functions
		/** @brief Gets the base position
		 * @return The base position
		 */
		Eigen::Vector3d getBasePosition() const;

		/** @brief Gets the base quaternion
		 * @return The base quaternion
		 */
		Eigen::Vector4d getBaseOrientation() const;

		/** @brief Gets the base RPY angles
		 * @return The base RPY angles
		 */
		Eigen::Vector3d getBaseRPY() const;

		/** @brief Gets the quaternion of the horizontal frame
		 * @return The quaternion of the horizontal frame
		 */
		Eigen::Vector4d getHorizontalOrientation() const;

		/** @brief Gets the RPY angles of the horizontal frame
		 * @return the RPY angles of the horizontal frame
		 */
		Eigen::Vector3d getHorizontalRPY() const;

		/** @brief Gets the base velocity expressed in the world frame
		 * @return The base velocity expressed in the world frame
		 */
		Eigen::Vector3d getBaseVelocity_W() const;

		/** @brief Gets the base velocity of the base frame
		 * @return The base velocity of the horizontal frame
		 */
		Eigen::Vector3d getBaseVelocity_B() const;

		/** @brief Gets the base velocity expressed in the horizontal frame
		 * @return The base velocity expressed in the horizontal frame
		 */
		Eigen::Vector3d getBaseVelocity_H() const;

		/** @brief Gets the base angular velocity expressed in the world frame
		 * @return The base angular velocity expressed in the world frame
		 */
		Eigen::Vector3d getBaseAngularVelocity_W() const;

		/** @brief Gets the base angular velocity expressed in the base frame
		 * @return The base angular velocity expressed in the base frame
		 */
		Eigen::Vector3d getBaseAngularVelocity_B() const;

		/** @brief Gets the base angular velocity expressed in the horizontal frame
		 * @return The base angular velocity expressed in the horizontal frame
		 */
		Eigen::Vector3d getBaseAngularVelocity_H() const;

		/** @brief Gets the base RPY velocity expressed in the world frame
		 * @return The base RPY velocity expressed in the world frame
		 */
		Eigen::Vector3d getBaseRPYVelocity_W() const;

		/** @brief Gets the base acceleration expressed in the world frame
		 * @return The base acceleration expressed in the world frame
		 */
		Eigen::Vector3d getBaseAcceleration_W() const;

		/** @brief Gets the base acceleration expressed in the base frame
		 * @return The base acceleration expressed in the base frame
		 */
		Eigen::Vector3d getBaseAcceleration_B() const;

		/** @brief Gets the base acceleration expressed in the horizontal frame
		 * @return The base acceleration expressed in the horizontal frame
		 */
		Eigen::Vector3d getBaseAcceleration_H() const;

		/** @brief Gets the base angular acceleration expressed in the world frame
		 * @return The base angular acceleration expressed in the world frame
		 */
		Eigen::Vector3d getBaseAngularAcceleration_W() const;

		/** @brief Gets the base angular acceleration expressed in the base frame
		 * @return The base angular acceleration expressed in the base frame
		 */
		Eigen::Vector3d getBaseAngularAcceleration_B() const;

		/** @brief Gets the base angular acceleration expressed in the horizontal frame
		 * @return The base angular acceleration expressed in the horizontal frame
		 */
		Eigen::Vector3d getBaseAngularAcceleration_H() const;

		/** @brief Gets the base RPY acceleration expressed in the world frame
		 * @return The base RPY acceleration expressed in the world frame
		 */
		Eigen::Vector3d getBaseRPYAcceleration_W() const;


		// Joint state getter functions
		/** @brief Gets the joint positions
		 * @return The joint positions
		 */
		const Eigen::VectorXd& getJointPosition() const;

		/** @brief Gets the joint position given its index
		 * @param[in] index The joint index
		 * @return The joint position
		 */
		const double& getJointPosition(const unsigned int& index) const;

		/** @brief Gets the joint velocities
		 * @return The joint velocities
		 */
		const Eigen::VectorXd& getJointVelocity() const;

		/** @brief Gets the joint velocity given its index
		 * @param[in] index The joint index
		 * @return The joint velocity
		 */
		const double& getJointVelocity(const unsigned int& index) const;

		/** @brief Gets the joint accelerations
		 * @return The joint accelerations
		 */
		const Eigen::VectorXd& getJointAcceleration() const;

		/** @brief Gets the joint acceleration given its index
		 * @param[in] index The joint index
		 * @return The joint acceleration
		 */
		const double& getJointAcceleration(const unsigned int& index) const;

		/** @brief Gets the joint efforts
		 * @return The joint efforts
		 */
		const Eigen::VectorXd& getJointEffort() const;

		/** @brief Gets the joint effort given its index
		 * @param[in] index The joint index
		 * @return The joint effort
		 */
		const double& getJointEffort(const unsigned int& index) const;

		/** @brief Gets the number of joints
		 * @return The number of joints
		 */
		const unsigned int getJointDoF() const;


		// Contact state getter functions
		/** @brief Gets the contact position expressed the world frame
		 * @param[in] pos_it The contact position iterator
		 * @return The contact position expressed in the world frame
		 */
		Eigen::VectorXd getContactPosition_W(ContactIterator pos_it) const;

		/** @brief Gets the contact position expressed the world frame
		 * @param[in] The contact name
		 * @return The contact position expressed in the world frame
		 */
		Eigen::VectorXd getContactPosition_W(const std::string& name) const;

		/** @brief Gets all contact positions expressed the world frame
		 * @return All contact positions expressed in the world frame
		 */
		Eigen::VectorXdMap getContactPosition_W() const;

		/** @brief Gets the contact position expressed the base frame
		 * @param[in] pos_it The contact position iterator
		 * @return The contact position expressed in the base frame
		 */
		const Eigen::VectorXd& getContactPosition_B(ContactIterator pos_it) const;

		/** @brief Gets the contact position expressed the base frame
		 * @param[in] The contact name
		 * @return The contact position expressed in the base frame
		 */
		const Eigen::VectorXd& getContactPosition_B(const std::string& name) const;

		/** @brief Gets all contact positions expressed the world frame
		 * @return All contact positions expressed in the world frame
		 */
		const Eigen::VectorXdMap& getContactPosition_B() const;

		/** @brief Gets the contact position expressed the horizontal frame
		 * @param[in] pos_it The contact position iterator
		 * @return The contact position expressed in the horizontal frame
		 */
		Eigen::VectorXd getContactPosition_H(ContactIterator pos_it) const;

		/** @brief Gets the contact position expressed the horizontal frame
		 * @param[in] name The contact name
		 * @return The contact position expressed in the horizontal frame
		 */
		Eigen::VectorXd getContactPosition_H(const std::string& name) const;

		/** @brief Gets all contact positions expressed the horizontal frame
		 * @return All contact positions expressed in the horizontal frame
		 */
		Eigen::VectorXdMap getContactPosition_H() const;

		/** @brief Gets the contact velocity expressed the world frame
		 * @param[in] vel_it The contact velocity iterator
		 * @return The contact velocity expressed in the world frame
		 */
		Eigen::VectorXd getContactVelocity_W(ContactIterator vel_it) const;

		/** @brief Gets the contact velocity expressed the world frame
		 * @param[in] name The contact name
		 * @return The contact velocity expressed in the world frame
		 */
		Eigen::VectorXd getContactVelocity_W(const std::string& name) const;

		/** @brief Gets all contact velocities expressed the world frame
		 * @return All contact velocities expressed in the world frame
		 */
		Eigen::VectorXdMap getContactVelocity_W() const;

		/** @brief Gets the contact velocity expressed the base frame
		 * @param[in] vel_it The contact velocity iterator
		 * @return The contact velocity expressed in the base frame
		 */
		const Eigen::VectorXd& getContactVelocity_B(ContactIterator vel_it) const;

		/** @brief Gets the contact velocity expressed the base frame
		 * @param[in] name The contact name
		 * @return The contact velocity expressed in the base frame
		 */
		const Eigen::VectorXd& getContactVelocity_B(const std::string& name) const;

		/** @brief Gets all contact velocities expressed the base frame
		 * @return All contact velocity expressed in the base frame
		 */
		const Eigen::VectorXdMap& getContactVelocity_B() const;

		/** @brief Gets the contact velocity expressed the horizontal frame
		 * @param[in] vel_it The contact velocity iterator
		 * @return The contact velocity expressed in the horizontal frame
		 */
		Eigen::VectorXd getContactVelocity_H(ContactIterator vel_it) const;

		/** @brief Gets the contact velocity expressed the horizontal frame
		 * @param[in] name The contact name
		 * @return The contact velocity expressed in the horizontal frame
		 */
		Eigen::VectorXd getContactVelocity_H(const std::string& name) const;

		/** @brief Gets all contact velocity expressed the horizontal frame
		 * @return All contact velocity expressed in the horizontal frame
		 */
		Eigen::VectorXdMap getContactVelocity_H() const;

		/** @brief Gets the contact acceleration expressed the world frame
		 * @param[in] acc_it The contact acceleration iterator
		 * @return The contact acceleration expressed in the world frame
		 */
		Eigen::VectorXd getContactAcceleration_W(ContactIterator acc_it) const;

		/** @brief Gets the contact acceleration expressed the world frame
		 * @param[in] name The contact name
		 * @return The contact acceleration expressed in the world frame
		 */
		Eigen::VectorXd getContactAcceleration_W(const std::string& name) const;

		/** @brief Gets all contact accelerations expressed the world frame
		 * @return All contact accelerations expressed in the world frame
		 */
		Eigen::VectorXdMap getContactAcceleration_W() const;

		/** @brief Gets the contact acceleration expressed the base frame
		 * @param[in] acc_it The contact acceleration iterator
		 * @return The contact acceleration expressed in the base frame
		 */
		const Eigen::VectorXd& getContactAcceleration_B(ContactIterator acc_it) const;

		/** @brief Gets the contact acceleration expressed the base frame
		 * @param[in] name The contact name
		 * @return The contact acceleration expressed in the base frame
		 */
		const Eigen::VectorXd& getContactAcceleration_B(const std::string& name) const;

		/** @brief Gets all contact acceleration expressed the base frame
		 * @return All contact accelerations expressed in the base frame
		 */
		const Eigen::VectorXdMap& getContactAcceleration_B() const;

		/** @brief Gets the contact acceleration expressed the horizontal frame
		 * @param[in] acc_it The contact acceleration iterator
		 * @return The contact acceleration expressed in the horizontal frame
		 */
		Eigen::VectorXd getContactAcceleration_H(ContactIterator acc_it) const;

		/** @brief Gets the contact acceleration expressed the horizontal frame
		 * @param[in] name The contact name
		 * @return The contact acceleration expressed in the horizontal frame
		 */
		Eigen::VectorXd getContactAcceleration_H(const std::string& name) const;

		/** @brief Gets the contact acceleration expressed the horizontal frame
		 * @return All contact accelerations expressed in the horizontal frame
		 */
		Eigen::VectorXdMap getContactAcceleration_H() const;

		/** @brief Gets the contact wrench expressed the base frame
		 * @param[in] name The contact name
		 * @return The contact wrench expressed in the base frame
		 */
		const Eigen::Vector6d& getContactWrench_B(const std::string& name) const;		

		/** @brief Gets all contact wrenches expressed the base frame
		 * @return All contact wrenches expressed in the base frame
		 */
		const Eigen::Vector6dMap& getContactWrench_B() const;

		/** @brief Gets the contact condition (active or inactive)
		 * @param[in] name The contact name
		 * @param[in] threshold Force threshold for detecting contact condition
		 * @return True for active contact conditions, false for inactive ones
		 */
		bool getContactCondition(const std::string& name,
								 const double& force_threshold) const;

		/** @brief Gets the contact condition (active or inactive)
		 * @param[in] name The contact name
		 * @return True for active contact conditions, false for inactive ones
		 */
		bool getContactCondition(const std::string& name) const;

		/** @brief Sets the time value
		 * @param[in] time The time value
		 */
		void setTime(const double& time);

		// Base state setter functions
		/** @brief Sets the base position expressed the world frame
		 * @param[in] pos The base position expressed in the world frame
		 */
		void setBasePosition(const Eigen::Vector3d& pos);

		/** @brief Sets the base quaternion expressed the world frame
		 * @param[in] q The base quaternion expressed in the world frame
		 */
		void setBaseOrientation(const Eigen::Vector4d& q);

		/** @brief Sets the base RPY angles expressed the world frame
		 * @param[in] rpy The base RPY angles expressed in the world frame
		 */
		void setBaseRPY(const Eigen::Vector3d& rpy);

		/** @brief Sets the base velocity expressed the world frame
		 * @param[in] vel_W The base velocity expressed in the world frame
		 */
		void setBaseVelocity_W(const Eigen::Vector3d& vel_W);

		/** @brief Sets the base velocity expressed the base frame
		 * @param[in] vel_B The base velocity expressed in the base frame
		 */
		void setBaseVelocity_B(const Eigen::Vector3d& vel_B);

		/** @brief Sets the base velocity expressed the horizontal frame
		 * @param[in] vel_H The base velocity expressed in the horizontal frame
		 */
		void setBaseVelocity_H(const Eigen::Vector3d& vel_H);

		/** @brief Sets the base angular velocity expressed the world frame
		 * @param[in] rate_W The base angular velocity expressed in the world frame
		 */
		void setBaseAngularVelocity_W(const Eigen::Vector3d& rate_W);

		/** @brief Sets the base angular velocity expressed the base frame
		 * @param[in] rate_B The base angular velocity expressed in the base frame
		 */
		void setBaseAngularVelocity_B(const Eigen::Vector3d& rate_B);

		/** @brief Sets the base angular velocity expressed the horizontal frame
		 * @param[in] rate_H The base angular velocity expressed in the horizontal frame
		 */
		void setBaseAngularVelocity_H(const Eigen::Vector3d& rate_H);

		/** @brief Sets the base RPY velocity expressed the world frame
		 * @param[in] rpy_rate The base RPY velocity expressed in the world frame
		 */
		void setBaseRPYVelocity_W(const Eigen::Vector3d& rpy_rate);

		/** @brief Sets the base acceleration expressed the world frame
		 * @param[in] acc_W The base acceleration expressed in the world frame
		 */
		void setBaseAcceleration_W(const Eigen::Vector3d& acc_W);

		/** @brief Sets the base acceleration expressed the base frame
		 * @param[in] acc_B The base acceleration expressed in the base frame
		 */
		void setBaseAcceleration_B(const Eigen::Vector3d& acc_B);

		/** @brief Sets the base acceleration expressed the horizontal frame
		 * @param[in] acc_H The base acceleration expressed in the horizontal frame
		 */
		void setBaseAcceleration_H(const Eigen::Vector3d& acc_H);

		/** @brief Sets the base angular acceleration expressed the world frame
		 * @param[in] rotacc_W The base angular acceleration expressed in the world frame
		 */
		void setBaseAngularAcceleration_W(const Eigen::Vector3d& rotacc_W);

		/** @brief Sets the base angular acceleration expressed the base frame
		 * @param[in] rotacc_B The base angular acceleration expressed in the base frame
		 */
		void setBaseAngularAcceleration_B(const Eigen::Vector3d& rotacc_B);

		/** @brief Sets the base angular acceleration expressed the horizontal frame
		 * @param[in] rotacc_H The base angular acceleration expressed in the horizontal frame
		 */
		void setBaseAngularAcceleration_H(const Eigen::Vector3d& rotacc_H);

		/** @brief Sets the base RPY acceleration expressed the world frame
		 * @param[in] rpy_rate The base RPY acceleration expressed in the world frame
		 */
		void setBaseRPYAcceleration_W(const Eigen::Vector3d& rpy_rate);


		// Joint state setter functions
		/** @brief Sets the joint positions
		 * @param[in] pos The joint positions
		 */
		void setJointPosition(const Eigen::VectorXd& pos);

		/** @brief Sets the joint position given its index
		 * @param[in] pos The joint position
		 * @param[in] index The joint index
		 */
		void setJointPosition(const double& pos,
							  const unsigned int& index);

		/** @brief Sets the joint velocites
		 * @param[in] vel The joint velocities
		 */
		void setJointVelocity(const Eigen::VectorXd& vel);

		/** @brief Sets the joint velocity given its index
		 * @param[in] vel The joint velocity
		 * @param[in] index The joint index
		 */
		void setJointVelocity(const double& vel,
							  const unsigned int& index);

		/** @brief Sets the joint accelerations
		 * @param[in] acc The joint acceleration
		 */
		void setJointAcceleration(const Eigen::VectorXd& acc);

		/** @brief Sets the joint acceleration given its index
		 * @param[in] acc The joint acceleration
		 * @param[in] index The joint index
		 */
		void setJointAcceleration(const double& acc,
								  const unsigned int& index);

		/** @brief Sets the joint efforts
		 * @param[in] eff The joint efforts
		 */
		void setJointEffort(const Eigen::VectorXd& eff);

		/** @brief Sets the joint effort given its index
		 * @param[in] eff The joint effort
		 * @param[in] index The joint index
		 */
		void setJointEffort(const double& eff,
							const unsigned int& index);

		/** @brief Sets the number of joints
		 * @param[in] num_joints Number of joints
		 */
		void setJointDoF(const unsigned int& num_joints);


		// Contact state setter functions		
		/** @brief Sets the contact position expressed the world frame
		 * @param[in] pos_it The contact position iterator
		 */
		void setContactPosition_W(ContactIterator pos_it);

		/** @brief Sets the contact position expressed the world frame
		 * @param[in] name The contact name
		 * @param[in] pos_W The contact position
		 */
		void setContactPosition_W(const std::string& name,
								  const Eigen::VectorXd& pos_W);

		/** @brief Sets all contact positions expressed the world frame
		 * @param[in] pos_W All contact positions
		 */
		void setContactPosition_W(const Eigen::VectorXdMap& pos_W);
		
		/** @brief Sets the contact position expressed the base frame
		 * @param[in] pos_it The contact position iterator
		 */
		void setContactPosition_B(ContactIterator pos_it);

		/** @brief Sets the contact position expressed the base frame
		 * @param[in] name The contact name
		 * @param[in] pos_B The contact position
		 */
		void setContactPosition_B(const std::string& name,
								  const Eigen::VectorXd& pos_B);

		/** @brief Sets all contact positions expressed the base frame
		 * @param[in] pos_B All contact positions
		 */
		void setContactPosition_B(const Eigen::VectorXdMap& pos_B);
		
		/** @brief Sets the contact position expressed the horizontal frame
		 * @param[in] pos_it The contact position iterator
		 */
		void setContactPosition_H(ContactIterator pos_it);

		/** @brief Sets the contact position expressed the horizontal frame
		 * @param[in] name The contact name
		 * @param[in] pos_H The contact position
		 */
		void setContactPosition_H(const std::string& name,
								  const Eigen::VectorXd& pos_H);

		/** @brief Sets the contact positions expressed the horizontal frame
		 * @param[in] pos_H All contact positions
		 */
		void setContactPosition_H(const Eigen::VectorXdMap& pos_H);
		
		/** @brief Sets the contact velocity expressed the world frame
		 * @param[in] vel_it The contact velocity iterator
		 */
		void setContactVelocity_W(ContactIterator vel_it);

		/** @brief Sets the contact velocity expressed the world frame
		 * @param[in] name The contact name
		 * @param[in] vel_W The contact velocity
		 */
		void setContactVelocity_W(const std::string& name,
								  const Eigen::VectorXd& vel_W);

		/** @brief Sets the contact velocities expressed the world frame
		 * @param[in] vel_W All contact velocities
		 */
		void setContactVelocity_W(const Eigen::VectorXdMap& vel_W);

		/** @brief Sets the contact velocity expressed the base frame
		 * @param[in] vel_it The contact velocity
		 */
		void setContactVelocity_B(ContactIterator vel_it);

		/** @brief Sets the contact velocity expressed the base frame
		 * @param[in] name The contact name
		 * @param[in] vel_B The contact velocity
		 */
		void setContactVelocity_B(const std::string& name,
								  const Eigen::VectorXd& vel_B);

		/** @brief Sets all contact velocities expressed the base frame
		 * @param[in] vel_B All contact velocities
		 */
		void setContactVelocity_B(const Eigen::VectorXdMap& vel_B);

		/** @brief Sets the contact velocity expressed the horizontal frame
		 * @param[in] vel_it The contact velocity
		 */
		void setContactVelocity_H(ContactIterator vel_it);

		/** @brief Sets the contact velocity expressed the horizontal frame
		 * @param[in] name The contact name
		 * @param[in] vel_H The contact velocity
		 */
		void setContactVelocity_H(const std::string& name,
								  const Eigen::VectorXd& vel_H);

		/** @brief Sets all contact velocities expressed the horizontal frame
		 * @param[in] vel_H All contact velocities
		 */
		void setContactVelocity_H(const Eigen::VectorXdMap& vel_H);
		
		/** @brief Sets the contact acceleration expressed the world frame
		 * @param[in] acc_W The contact acceleration
		 */
		void setContactAcceleration_W(ContactIterator acc_it);

		/** @brief Sets the contact acceleration expressed the world frame
		 * @param[in] name The contact name
		 * @param[in] acc_W The contact acceleration
		 */
		void setContactAcceleration_W(const std::string& name,
									  const Eigen::VectorXd& acc_W);

		/** @brief Sets all contact accelerations expressed the world frame
		 * @param[in] acc_W All contact accelerations
		 */
		void setContactAcceleration_W(const Eigen::VectorXdMap& acc_W);

		/** @brief Sets the contact acceleration expressed the base frame
		 * @param[in] acc_it The contact acceleration iterator
		 */
		void setContactAcceleration_B(ContactIterator acc_it);

		/** @brief Sets the contact acceleration expressed the base frame
		 * @param[in] name The contact name
		 * @param[in] acc_B The contact acceleration
		 */
		void setContactAcceleration_B(const std::string& name,
									  const Eigen::VectorXd& acc_B);

		/** @brief Sets all contact accelerations expressed the base frame
		 * @param[in] acc_B All contact accelerations
		 */
		void setContactAcceleration_B(const Eigen::VectorXdMap& acc_B);

		/** @brief Sets the contact acceleration expressed the horizontal frame
		 * @param[in] acc_it The contact acceleration iterator
		 */
		void setContactAcceleration_H(ContactIterator acc_it);

		/** @brief Sets the contact acceleration expressed the horizontal frame
		 * @param[in] name The contact name
		 * @param[in] acc_H The contact acceleration
		 */
		void setContactAcceleration_H(const std::string& name,
									  const Eigen::VectorXd& acc_H);

		/** @brief Sets all contact accelerations expressed the horizontal frame
		 * @param[in] acc_H All contact accelerations
		 */
		void setContactAcceleration_H(const Eigen::VectorXdMap& acc_H);

		/** @brief Sets the contact wrench expressed the base frame
		 * @param[in] name The contact name
		 * @param[in] eff_B The contact wrench
		 */
		void setContactWrench_B(const std::string& name,
							    const Eigen::Vector6d& eff);

		/** @brief Sets all contact wrenches expressed the base frame
		 * @param[in] eff_B All contact wrenches
		 */
		void setContactWrench_B(const Eigen::Vector6dMap& eff_B);

		/** @brief Sets the contact condition (active or inactive)
		 * @param name The contact name
		 * @param condition True for active conditions, and false for inactive
		 */
		void setContactCondition(const std::string& name,
								 const bool& condition);

		/** @brief Internal whole-body state variables expressed with the
		 * above mentioned convention */
		double time;
		double duration;
		Eigen::Vector7d base_pos;
		Eigen::Vector6d base_vel;
		Eigen::Vector6d base_acc;
		Eigen::Vector6d base_eff;
		Eigen::VectorXd joint_pos;
		Eigen::VectorXd joint_vel;
		Eigen::VectorXd joint_acc;
		Eigen::VectorXd joint_eff;
		Eigen::VectorXdMap contact_pos;
		Eigen::VectorXdMap contact_vel;
		Eigen::VectorXdMap contact_acc;
		Eigen::Vector6dMap contact_eff;


	private:
		/** @brief Number of joints */
		unsigned int num_joints_;

		/** @brief Frame transformations */
		math::FrameTF frame_tf_;

		/** @brief Default value for joint states that don't exist */
		double default_joint_value_;

		/** @brief Null vectors for missed contact states */
		Eigen::VectorXd null_3dvector_;
		Eigen::Vector6d null_6dvector_;
};

/** @brief Defines a whole-body trajectory */
typedef std::vector<WholeBodyState> WholeBodyTrajectory;

} //@namespace dwl

#endif
