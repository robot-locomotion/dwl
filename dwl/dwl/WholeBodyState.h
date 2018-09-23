#ifndef DWL__WHOLE_BODY_STATE__H
#define DWL__WHOLE_BODY_STATE__H

#include <map>
#include <vector>

#include <dwl/utils/LieGroup.h>
#include <dwl/utils/RigidBodyDynamics.h>

#define ZERO_VECTOR3D Eigen::Vector3d::Zero()
#define MAX_VECTOR3D 2e19 * Eigen::Vector3d::Ones()
#define INACTIVE_CONTACT ZERO_VECTOR3D
#define ACTIVE_CONTACT MAX_VECTOR3D


namespace dwl
{

/**
 * @brief The WholeBodyState class
 * This class describes the whole-body state of the robot, including:
 * <ul>
 *    <li>time, in seconds</li>
 *    <li>Base SE3 configuration, expressed in the world frame</li>
 *    <li>Base velocity, expressed in the world frame</li>
 *    <li>Base acceleration, expressed in the world frame</li>
 *    <li>Joint positions, expressed with the urdf order</li>
 *    <li>Joint velocities, expressed with the urdf order</li>
 *    <li>Joint accelerations, expressed with the urdf order</li>
 *    <li>Joint efforts, expressed with the urdf order</li>
 *    <li>Contact SE3 configurations, expressed in the base frame</li>
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
 * 	 <li>base_pos [x, y, z, qx, qy, qz, qw] expressed in the world frame </li>
 * 	 <li>base_vel [x, y, z, wx, wy, wz,] expressed in the world frame </li>
 * 	 <li>base_acc [x, y, z, wx, wy, wz] expressed in the world frame </li>
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
		typedef dwl::SE3Map::const_iterator SE3Iterator;
		typedef dwl::MotionMap::const_iterator MotionIterator;
		typedef dwl::ForceMap::const_iterator ForceIterator;

		/** @brief Constructor function */
		WholeBodyState(unsigned int num_joints = 0);

		/** @brief Destructor function */
		~WholeBodyState();

		// Time getter function
		/** @brief Gets the time value
		 * @return Time
		 */
		const double& getTime() const;


		// Base state getter functions
		/**
		 * @brief Gets the base SE3
		 * @return Base SE3
		 */
		const dwl::SE3& getBaseSE3() const;

		/**
		 * @brief Gets the base SE3 of the horizontal frame
		 * @return Base SE3 of the horizontal frame
		 */
		const dwl::SE3& getBaseSE3_H();

		/** @brief Gets the base velocity expressed in the world frame
		 * @return Base velocity expressed in the world frame
		 */
		const dwl::Motion& getBaseVelocity_W() const;

		/** @brief Gets the base velocity of the base frame
		 * @return Base velocity of the horizontal frame
		 */
		const dwl::Motion& getBaseVelocity_B();

		/** @brief Gets the base velocity expressed in the horizontal frame
		 * @return Base velocity expressed in the horizontal frame
		 */
		const dwl::Motion& getBaseVelocity_H();

		/** @brief Gets the base RPY velocity expressed in the world frame
		 * @return Base RPY velocity expressed in the world frame
		 */
		const Eigen::Vector3d& getBaseRPYVelocity_W();

		/** @brief Gets the base acceleration expressed in the world frame
		 * @return Base acceleration expressed in the world frame
		 */
		const dwl::Motion& getBaseAcceleration_W() const;

		/** @brief Gets the base acceleration expressed in the base frame
		 * @return Base acceleration expressed in the base frame
		 */
		const dwl::Motion& getBaseAcceleration_B();

		/** @brief Gets the base acceleration expressed in the horizontal frame
		 * @return Base acceleration expressed in the horizontal frame
		 */
		const dwl::Motion& getBaseAcceleration_H();

		/** @brief Gets the base RPY acceleration expressed in the world frame
		 * @return Base RPY acceleration expressed in the world frame
		 */
		const Eigen::Vector3d& getBaseRPYAcceleration_W();

		/** @brief Gets the base wrench expressed in the world frame
		 * @return Base wrench expressed in the world frame
		 */
		const dwl::Force& getBaseWrench_W();


		// Joint state getter functions
		/** @brief Gets the joint positions
		 * @return Joint positions
		 */
		const Eigen::VectorXd& getJointPosition() const;

		/** @brief Gets the joint position given its index
		 * @param[in] index Joint index
		 * @return Joint position
		 */
		const double& getJointPosition(const unsigned int& index) const;

		/** @brief Gets the joint velocities
		 * @return Joint velocities
		 */
		const Eigen::VectorXd& getJointVelocity() const;

		/** @brief Gets the joint velocity given its index
		 * @param[in] index Joint index
		 * @return Joint velocity
		 */
		const double& getJointVelocity(const unsigned int& index) const;

		/** @brief Gets the joint accelerations
		 * @return Joint accelerations
		 */
		const Eigen::VectorXd& getJointAcceleration() const;

		/** @brief Gets the joint acceleration given its index
		 * @param[in] index Joint index
		 * @return Joint acceleration
		 */
		const double& getJointAcceleration(const unsigned int& index) const;

		/** @brief Gets the joint efforts
		 * @return Joint efforts
		 */
		const Eigen::VectorXd& getJointEffort() const;

		/** @brief Gets the joint effort given its index
		 * @param[in] index Joint index
		 * @return Joint effort
		 */
		const double& getJointEffort(const unsigned int& index) const;

		/** @brief Gets the number of joints
		 * @return The number of joints
		 */
		const unsigned int getJointDoF() const;


		// Contact state getter functions
		/** @brief Gets the contact SE3 expressed the world frame
		 * @param[in] it Contact SE3 iterator
		 * @return Contact SE3 expressed in the world frame
		 */
		const dwl::SE3& getContactSE3_W(SE3Iterator it);

		/** @brief Gets the contact SE3 expressed the world frame
		 * @param[in] Contact name
		 * @return Contact SE3 expressed in the world frame
		 */
		const dwl::SE3& getContactSE3_W(const std::string& name);

		/** @brief Gets all contact positions expressed the world frame
		 * @return All contact positions expressed in the world frame
		 */
		dwl::SE3Map getContactSE3_W();

		/** @brief Gets the contact SE3 expressed the base frame
		 * @param[in] it Contact SE3 iterator
		 * @return Contact SE3 expressed in the base frame
		 */
		const dwl::SE3& getContactSE3_B(SE3Iterator it) const;

		/** @brief Gets the contact SE3 expressed the base frame
		 * @param[in] Contact name
		 * @return Contact SE3 expressed in the base frame
		 */
		const dwl::SE3& getContactSE3_B(const std::string& name) const;

		/** @brief Gets all contact SE3s expressed the world frame
		 * @return All contact positions expressed in the world frame
		 */
		const dwl::SE3Map& getContactSE3_B() const;

		/** @brief Gets the contact position expressed the horizontal frame
		 * @param[in] it Contact SE3 iterator
		 * @return The contact position expressed in the horizontal frame
		 */
		const dwl::SE3& getContactSE3_H(SE3Iterator it);

		/** @brief Gets the contact position expressed the horizontal frame
		 * @param[in] name Contact name
		 * @return Contact SE3 expressed in the horizontal frame
		 */
		const dwl::SE3& getContactSE3_H(const std::string& name);

		/** @brief Gets all contact positions expressed the horizontal frame
		 * @return All contact SE3s expressed in the horizontal frame
		 */
		dwl::SE3Map getContactSE3_H();

		/** @brief Gets the contact velocity expressed the world frame
		 * @param[in] it Contact velocity iterator
		 * @return Contact velocity expressed in the world frame
		 */
		const dwl::Motion& getContactVelocity_W(MotionIterator it);

		/** @brief Gets the contact velocity expressed the world frame
		 * @param[in] name Contact name
		 * @return Contact velocity expressed in the world frame
		 */
		const dwl::Motion& getContactVelocity_W(const std::string& name);

		/** @brief Gets all contact velocities expressed the world frame
		 * @return All contact velocities expressed in the world frame
		 */
		dwl::MotionMap getContactVelocity_W();

		/** @brief Gets the contact velocity expressed the base frame
		 * @param[in] it Contact velocity iterator
		 * @return Contact velocity expressed in the base frame
		 */
		const dwl::Motion& getContactVelocity_B(MotionIterator it) const;

		/** @brief Gets the contact velocity expressed the base frame
		 * @param[in] name Contact name
		 * @return Contact velocity expressed in the base frame
		 */
		const dwl::Motion& getContactVelocity_B(const std::string& name) const;

		/** @brief Gets all contact velocities expressed the base frame
		 * @return All contact velocity expressed in the base frame
		 */
		const dwl::MotionMap& getContactVelocity_B() const;

		/** @brief Gets the contact velocity expressed the horizontal frame
		 * @param[in] it Contact velocity iterator
		 * @return Contact velocity expressed in the horizontal frame
		 */
		const dwl::Motion& getContactVelocity_H(MotionIterator it);

		/** @brief Gets the contact velocity expressed the horizontal frame
		 * @param[in] name Contact name
		 * @return Contact velocity expressed in the horizontal frame
		 */
		const dwl::Motion& getContactVelocity_H(const std::string& name);

		/** @brief Gets all contact velocity expressed the horizontal frame
		 * @return All contact velocity expressed in the horizontal frame
		 */
		dwl::MotionMap getContactVelocity_H();

		/** @brief Gets the contact acceleration expressed the world frame
		 * @param[in] it Contact acceleration iterator
		 * @return Contact acceleration expressed in the world frame
		 */
		const dwl::Motion& getContactAcceleration_W(MotionIterator it);

		/** @brief Gets the contact acceleration expressed the world frame
		 * @param[in] name Contact name
		 * @return Contact acceleration expressed in the world frame
		 */
		const dwl::Motion& getContactAcceleration_W(const std::string& name);

		/** @brief Gets all contact accelerations expressed the world frame
		 * @return All contact accelerations expressed in the world frame
		 */
		dwl::MotionMap getContactAcceleration_W();

		/** @brief Gets the contact acceleration expressed the base frame
		 * @param[in] it The contact acceleration iterator
		 * @return Contact acceleration expressed in the base frame
		 */
		const dwl::Motion& getContactAcceleration_B(MotionIterator it) const;

		/** @brief Gets the contact acceleration expressed the base frame
		 * @param[in] name Contact name
		 * @return Contact acceleration expressed in the base frame
		 */
		const dwl::Motion& getContactAcceleration_B(const std::string& name) const;

		/** @brief Gets all contact acceleration expressed the base frame
		 * @return All contact accelerations expressed in the base frame
		 */
		const dwl::MotionMap& getContactAcceleration_B() const;

		/** @brief Gets the contact acceleration expressed the horizontal frame
		 * @param[in] it Contact acceleration iterator
		 * @return Contact acceleration expressed in the horizontal frame
		 */
		const dwl::Motion& getContactAcceleration_H(MotionIterator it);

		/** @brief Gets the contact acceleration expressed the horizontal frame
		 * @param[in] name Contact name
		 * @return Contact acceleration expressed in the horizontal frame
		 */
		const dwl::Motion& getContactAcceleration_H(const std::string& name);

		/** @brief Gets the contact acceleration expressed the horizontal frame
		 * @return All contact accelerations expressed in the horizontal frame
		 */
		dwl::MotionMap getContactAcceleration_H();

		/** @brief Gets the contact wrench expressed the base frame
		 * @param[in] name Contact name
		 * @return Contact wrench expressed in the base frame
		 */
		const dwl::Force& getContactWrench_B(const std::string& name) const;

		/** @brief Gets all contact wrenches expressed the base frame
		 * @return All contact wrenches expressed in the base frame
		 */
		const dwl::ForceMap& getContactWrench_B() const;

		/** @brief Gets the contact condition (active or inactive)
		 * @param[in] name Contact name
		 * @param[in] f_th Force threshold for detecting contact condition
		 * @return True for active contact conditions, false for inactive ones
		 */
		bool getContactCondition(const std::string& name,
								 const double& f_th = 0.) const;

		/** @brief Sets the time value
		 * @param[in] time The time value
		 */
		void setTime(const double& time);

		// Base state setter functions
		/** @brief Sets the base SE3 expressed the world frame
		 * @param[in] pos Base SE3 expressed in the world frame
		 */
		void setBaseSE3(const dwl::SE3& pos);

		/** @brief Sets the base velocity expressed the world frame
		 * @param[in] vel Base velocity expressed in the world frame
		 */
		void setBaseVelocity_W(const dwl::Motion& vel);

		/** @brief Sets the base velocity expressed the base frame
		 * @param[in] vel Base velocity expressed in the base frame
		 */
		void setBaseVelocity_B(const dwl::Motion& vel);

		/** @brief Sets the base velocity expressed the horizontal frame
		 * @param[in] vel Base velocity expressed in the horizontal frame
		 */
		void setBaseVelocity_H(const dwl::Motion& vel);

		/** @brief Sets the base RPY velocity expressed the world frame
		 * @param[in] rpy_rate Base RPY velocity expressed in the world frame
		 */
		void setBaseRPYVelocity_W(const Eigen::Vector3d& rpy_rate);

		/** @brief Sets the base acceleration expressed the world frame
		 * @param[in] acc Base acceleration expressed in the world frame
		 */
		void setBaseAcceleration_W(const dwl::Motion& acc);

		/** @brief Sets the base acceleration expressed the base frame
		 * @param[in] acc Base acceleration expressed in the base frame
		 */
		void setBaseAcceleration_B(const dwl::Motion& acc);

		/** @brief Sets the base acceleration expressed the horizontal frame
		 * @param[in] acc Base acceleration expressed in the horizontal frame
		 */
		void setBaseAcceleration_H(const dwl::Motion& acc);

		/** @brief Sets the base RPY acceleration expressed the world frame
		 * @param[in] rpy_rate Base RPY acceleration expressed in the world frame
		 */
		void setBaseRPYAcceleration_W(const Eigen::Vector3d& rpy_rate);


		// Joint state setter functions
		/** @brief Sets Joint positions
		 * @param[in] pos Joint positions
		 */
		void setJointPosition(const Eigen::VectorXd& pos);

		/** @brief Sets the joint position given its index
		 * @param[in] pos Joint position
		 * @param[in] index Joint index
		 */
		void setJointPosition(const double& pos,
							  const unsigned int& index);

		/** @brief Sets the joint velocities
		 * @param[in] vel Joint velocities
		 */
		void setJointVelocity(const Eigen::VectorXd& vel);

		/** @brief Sets the joint velocity given its index
		 * @param[in] vel Joint velocity
		 * @param[in] index Joint index
		 */
		void setJointVelocity(const double& vel,
							  const unsigned int& index);

		/** @brief Sets the joint accelerations
		 * @param[in] acc Joint acceleration
		 */
		void setJointAcceleration(const Eigen::VectorXd& acc);

		/** @brief Sets the joint acceleration given its index
		 * @param[in] acc Joint acceleration
		 * @param[in] index Joint index
		 */
		void setJointAcceleration(const double& acc,
								  const unsigned int& index);

		/** @brief Sets the joint efforts
		 * @param[in] eff Joint efforts
		 */
		void setJointEffort(const Eigen::VectorXd& eff);

		/** @brief Sets the joint effort given its index
		 * @param[in] eff Joint effort
		 * @param[in] index Joint index
		 */
		void setJointEffort(const double& eff,
							const unsigned int& index);

		/** @brief Sets the number of joints
		 * @param[in] num_joints Number of joints
		 */
		void setJointDoF(const unsigned int& num_joints);


		// Contact state setter functions
		/** @brief Sets the contact SE3 expressed the world frame
		 * @param[in] it Contact SE3 iterator
		 */
		void setContactSE3_W(SE3Iterator it);

		/** @brief Sets the contact SE3 expressed the world frame
		 * @param[in] name Contact name
		 * @param[in] pos Contact SE3
		 */
		void setContactSE3_W(const std::string& name,
							 const dwl::SE3& pos);

		/** @brief Sets all contact SE3s expressed the world frame
		 * @param[in] pos All contact SE3s
		 */
		void setContactSE3_W(const dwl::SE3Map& pos);

		/** @brief Sets the contact SE3 expressed the base frame
		 * @param[in] it Contact SE3 iterator
		 */
		void setContactSE3_B(SE3Iterator it);

		/** @brief Sets the contact SE3 expressed the base frame
		 * @param[in] name Contact name
		 * @param[in] pos Contact SE3
		 */
		void setContactSE3_B(const std::string& name,
							 const dwl::SE3& pos);

		/** @brief Sets all contact SE3s expressed the base frame
		 * @param[in] pos All contact SE3s
		 */
		void setContactSE3_B(const dwl::SE3Map& pos);

		/** @brief Sets the contact SE3 expressed the horizontal frame
		 * @param[in] it Contact SE3 iterator
		 */
		void setContactSE3_H(SE3Iterator it);

		/** @brief Sets the contact SE3 expressed the horizontal frame
		 * @param[in] name Contact name
		 * @param[in] pos Contact SE3 expressed in the horizontal frame
		 */
		void setContactSE3_H(const std::string& name,
							 const dwl::SE3& pos);

		/** @brief Sets the contact positions expressed the horizontal frame
		 * @param[in] pos All contact SE3s
		 */
		void setContactSE3_H(const dwl::SE3Map& pos);

		/** @brief Sets the contact velocity expressed the world frame
		 * @param[in] it Contact velocity iterator
		 */
		void setContactVelocity_W(MotionIterator it);

		/** @brief Sets the contact velocity expressed the world frame
		 * @param[in] name Contact name
		 * @param[in] vel Contact velocity expressed in the world frame
		 */
		void setContactVelocity_W(const std::string& name,
								  const dwl::Motion& vel);

		/** @brief Sets the contact velocities expressed the world frame
		 * @param[in] vel All contact velocities
		 */
		void setContactVelocity_W(const dwl::MotionMap& vel);

		/** @brief Sets the contact velocity expressed the base frame
		 * @param[in] it Contact velocity iterator
		 */
		void setContactVelocity_B(MotionIterator it);

		/** @brief Sets the contact velocity expressed the base frame
		 * @param[in] name Contact name
		 * @param[in] vel Contact velocity expressed in the base frame
		 */
		void setContactVelocity_B(const std::string& name,
								  const dwl::Motion& vel);

		/** @brief Sets all contact velocities expressed the base frame
		 * @param[in] vel All contact velocities
		 */
		void setContactVelocity_B(const dwl::MotionMap& vel);

		/** @brief Sets the contact velocity expressed the horizontal frame
		 * @param[in] it Contact velocity iterator
		 */
		void setContactVelocity_H(MotionIterator it);

		/** @brief Sets the contact velocity expressed the horizontal frame
		 * @param[in] name Contact name
		 * @param[in] vel Contact velocity expressed in the horizontal frame
		 */
		void setContactVelocity_H(const std::string& name,
								  const dwl::Motion& vel);

		/** @brief Sets all contact velocities expressed the horizontal frame
		 * @param[in] vel_H All contact velocities
		 */
		void setContactVelocity_H(const dwl::MotionMap& vel);

		/** @brief Sets the contact acceleration expressed the world frame
		 * @param[in] it Contact acceleration iterator
		 */
		void setContactAcceleration_W(MotionIterator it);

		/** @brief Sets the contact acceleration expressed the world frame
		 * @param[in] name Contact name
		 * @param[in] acc_W Contact acceleration expressed in the world frame
		 */
		void setContactAcceleration_W(const std::string& name,
									  const dwl::Motion& acc_W);

		/** @brief Sets all contact accelerations expressed the world frame
		 * @param[in] acc_W All contact accelerations
		 */
		void setContactAcceleration_W(const dwl::MotionMap& acc_W);

		/** @brief Sets the contact acceleration expressed the base frame
		 * @param[in] it Contact acceleration iterator
		 */
		void setContactAcceleration_B(MotionIterator it);

		/** @brief Sets the contact acceleration expressed the base frame
		 * @param[in] name Contact name
		 * @param[in] acc Contact acceleration expressed in the base frame
		 */
		void setContactAcceleration_B(const std::string& name,
									  const dwl::Motion& acc_B);

		/** @brief Sets all contact accelerations expressed the base frame
		 * @param[in] acc All contact accelerations expressed in the base frame
		 */
		void setContactAcceleration_B(const dwl::MotionMap& acc);

		/** @brief Sets the contact acceleration expressed the horizontal frame
		 * @param[in] it Contact acceleration iterator
		 */
		void setContactAcceleration_H(MotionIterator it);

		/** @brief Sets the contact acceleration expressed the horizontal frame
		 * @param[in] name Contact name
		 * @param[in] acc_H Contact acceleration expressed in the horizontal frame
		 */
		void setContactAcceleration_H(const std::string& name,
									  const dwl::Motion& acc_H);

		/** @brief Sets all contact accelerations expressed the horizontal frame
		 * @param[in] acc_H All contact accelerations
		 */
		void setContactAcceleration_H(const dwl::MotionMap& acc_H);

		/** @brief Sets the contact wrench expressed the base frame
		 * @param[in] name Contact name
		 * @param[in] f_B Contact wrench expressed in the base frame
		 */
		void setContactWrench_B(const std::string& name,
							    const dwl::Force& f_B);

		/** @brief Sets all contact wrenches expressed the base frame
		 * @param[in] f_B All contact wrenches
		 */
		void setContactWrench_B(const dwl::ForceMap& f_B);

		/** @brief Sets the contact condition (active or inactive)
		 * @param name Contact name
		 * @param condition True for active conditions, and false for inactive
		 */
		void setContactCondition(const std::string& name,
								 const bool& condition);

		/** @brief Internal whole-body state variables expressed with the
		 * above mentioned convention */
		double time;
		double duration;
		dwl::SE3 base_pos;
		dwl::Motion base_vel;
		dwl::Motion base_acc;
		dwl::Force base_eff;
		Eigen::VectorXd joint_pos;
		Eigen::VectorXd joint_vel;
		Eigen::VectorXd joint_acc;
		Eigen::VectorXd joint_eff;
		dwl::SE3Map contact_pos;
		dwl::MotionMap contact_vel;
		dwl::MotionMap contact_acc;
		dwl::ForceMap contact_eff;


	private:
		/** @brief Internal data to avoid dynamic memory allocation **/
		dwl::SE3 se3_;
		dwl::Motion motion_;
		Eigen::Vector3d vec3_;

		/** @brief Number of joints */
		unsigned int num_joints_;

		/** @brief Default value for joint states that don't exist */
		double default_joint_value_;

		/** @brief Null vectors for missed contact states */
		dwl::SE3 null_se3_;
		dwl::Motion null_motion_;
		dwl::Force null_force_;
};

/** @brief Defines a whole-body trajectory */
typedef std::vector<WholeBodyState> WholeBodyTrajectory;

} //@namespace dwl

#endif
