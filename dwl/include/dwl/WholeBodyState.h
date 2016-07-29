#ifndef DWL__WHOLE_BODY_STATE__H
#define DWL__WHOLE_BODY_STATE__H

#include <Eigen/Dense>
#include <map>
#include <vector>

#include <dwl/utils/FrameTF.h>
#include <dwl/utils/RigidBodyDynamics.h>


#define NO_WRENCH dwl::rbd::Vector6d::Zero()
#define MAX_WRENCH 2e19 * dwl::rbd::Vector6d::Ones()
#define INACTIVE_CONTACT NO_WRENCH
#define ACTIVE_CONTACT MAX_WRENCH


namespace dwl
{

/**
 * @brief The WholeBodyState class
 * This class incorporate the whole-body state of the robot, including:
 * <ul>
 *    <li>time, in seconds</li>
 *    <li>Base position, expressed in the world frame of reference</li>
 *    <li>Base velocity, expressed in the world frame</li>
 *    <li>Base acceleration, expressed in the world frame</li>
 *    <li>Base orientation, expressed in a world frame of reference</li>
 *    <li>Base rotation rate, expressed in the world frame</li>
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
 * 	 <li>base_pos [roll, pitch, yaw, x, y, z]</li>
 * 	 <li>base_vel [rate_x, rate_y, rate_z, x, y, z]</li>
 * 	 <li>base_acc [rotacc_x, rotacc_y, rotacc_y, x, y, z]</li>
 * 	 <li>joint_pos</li>
 * 	 <li>joint_vel</li>
 * 	 <li>joint_acc</li>
 * 	 <li>joint_eff</li>
 * 	 <li>contact_pos</li>
 * 	 <li>contact_vel</li>
 * 	 <li>contact_acc</li>
 * 	 <li>contact_eff</li>
 * 	 Note that these quantities have to be expressed in the above mentioned
 * 	 frames and orders.
 * </ul>
 */
class WholeBodyState
{
	public:
		/** @brief Constructor function */
		WholeBodyState(unsigned int num_joints = 0);

		/** @brief Destructor function */
		~WholeBodyState();

		// Base state getter functions
		/** @brief Gets the base position in the world frame */
		Eigen::Vector3d getBasePosition_W() const;

		/** @brief Gets the base orientation in the world frame */
		Eigen::Quaterniond getBaseOrientation_W() const;

		/** @brief Gets the base RPY angles in the world frame */
		Eigen::Vector3d getBaseRPY_W() const;

		/** @brief Gets the base orientation in the horizontal frame */
		Eigen::Quaterniond getBaseOrientation_H() const;

		/** @brief Gets the base RPY angles in the horizontal frame */
		Eigen::Vector3d getBaseRPY_H() const;

		/** @brief Gets the base velocity in the world frame */
		Eigen::Vector3d getBaseVelocity_W() const;

		/** @brief Gets the base velocity in the base frame */
		Eigen::Vector3d getBaseVelocity_B() const;

		/** @brief Gets the base velocity in the horizontal frame */
		Eigen::Vector3d getBaseVelocity_H() const;

		/** @brief Gets the base rotation rate in the world frame */
		Eigen::Vector3d getBaseRotationRate_W() const;

		/** @brief Gets the base rotation rate in the base frame */
		Eigen::Vector3d getBaseRotationRate_B() const;

		/** @brief Gets the base rotation rate in the horizontal frame */
		Eigen::Vector3d getBaseRotationRate_H() const;

		/** @brief Gets the base acceleration in the world frame */
		Eigen::Vector3d getBaseAcceleration_W() const;

		/** @brief Gets the base acceleration in the base frame */
		Eigen::Vector3d getBaseAcceleration_B() const;

		/** @brief Gets the base acceleration in the horizontal frame */
		Eigen::Vector3d getBaseAcceleration_H() const;

		/** @brief Gets the base rotation acceleration in the world frame */
		Eigen::Vector3d getBaseRotAcceleration_W() const;

		/** @brief Gets the base rotation acceleration in the base frame */
		Eigen::Vector3d getBaseRotAcceleration_B() const;

		/** @brief Gets the base rotation acceleration in the horizontal frame */
		Eigen::Vector3d getBaseRotAcceleration_H() const;

		// Base state setter functions
		/** @brief Sets the base position in the world frame */
		void setBasePosition_W(const Eigen::Vector3d& pos_W);

		/** @brief Sets the base orientation in the world frame */
		void setBaseOrientation_W(const Eigen::Quaterniond& orient_W);

		/** @brief Sets the base RPY angles in the world frame */
		void setBaseRPY_W(const Eigen::Vector3d& rpy_W);

		/** @brief Sets the base velocity in the world frame */
		void setBaseVelocity_W(const Eigen::Vector3d& vel_W);

		/** @brief Sets the base velocity in the base frame */
		void setBaseVelocity_B(const Eigen::Vector3d& vel_B);

		/** @brief Sets the base velocity in the horizontal frame */
		void setBaseVelocity_H(const Eigen::Vector3d& vel_H);

		/** @brief Sets the base rotation rate in the world frame */
		void setBaseRotationRate_W(const Eigen::Vector3d& rate_W);

		/** @brief Sets the base rotation rate in the base frame */
		void setBaseRotationRate_B(const Eigen::Vector3d& rate_B);

		/** @brief Sets the base rotation rate in the horizontal frame */
		void setBaseRotationRate_H(const Eigen::Vector3d& rate_H);

		/** @brief Sets the base acceleration in the world frame */
		void setBaseAcceleration_W(const Eigen::Vector3d& acc_W);

		/** @brief Sets the base acceleration in the base frame */
		void setBaseAcceleration_B(const Eigen::Vector3d& acc_B);

		/** @brief Sets the base acceleration in the horizontal frame */
		void setBaseAcceleration_H(const Eigen::Vector3d& acc_H);

		/** @brief Sets the base rotation acceleration in the world frame */
		void setBaseRotAcceleration_W(const Eigen::Vector3d& rotacc_W);

		/** @brief Sets the base rotation acceleration in the base frame */
		void setBaseRotAcceleration_B(const Eigen::Vector3d& rotacc_B);

		/** @brief Sets the base rotation acceleration in the horizontal frame */
		void setBaseRotAcceleration_H(const Eigen::Vector3d& rotacc_H);


		// Joint state getter functions
		/** @brief Gets the joint positions */
		const Eigen::VectorXd& getJointPosition() const;
		const double& getJointPosition(unsigned int index) const;

		/** @brief Gets the joint velocities */
		const Eigen::VectorXd& getJointVelocity() const;
		const double& getJointVelocity(unsigned int index) const;

		/** @brief Gets the joint accelerations */
		const Eigen::VectorXd& getJointAcceleration() const;
		const double& getJointAcceleration(unsigned int index) const;

		/** @brief Gets the joint effort */
		const Eigen::VectorXd& getJointEffort() const;
		const double& getJointEffort(unsigned int index) const;

		/** @brief Gets the number of joints */
		const unsigned int getJointDof() const;

		// Joint state setter functions
		/** @brief Sets the joint positions */
		void setJointPosition(const Eigen::VectorXd& pos);
		void setJointPosition(double pos,
							  unsigned int index);

		/** @brief Sets the joint velocities */
		void setJointVelocity(const Eigen::VectorXd& vel);
		void setJointVelocity(double vel,
							  unsigned int index);

		/** @brief Sets the joint accelerations */
		void setJointAcceleration(const Eigen::VectorXd& acc);
		void setJointAcceleration(double acc,
								  unsigned int index);

		/** @brief Sets the joint efforts */
		void setJointEffort(const Eigen::VectorXd& eff);
		void setJointEffort(double eff,
							unsigned int index);

		/**
		 * @brief Sets the number of joints
		 * @param unsigned int Number of joints
		 */
		void setJointDoF(unsigned int num_joints);


		// Contact state getter functions
		/** @brief Gets the contact positions in the base frame */
		const rbd::BodyVector& getContactPosition_B() const;
		const Eigen::VectorXd& getContactPosition_B(std::string name) const;

		/** @brief Gets the contact velocities in the base frame */
		const rbd::BodyVector& getContactVelocity_B() const;
		const Eigen::VectorXd& getContactVelocity_B(std::string name) const;

		/** @brief Gets the contact accelerations in the base frame */
		const rbd::BodyVector& getContactAcceleration_B() const;
		const Eigen::VectorXd& getContactAcceleration_B(std::string name) const;

		/** @brief Gets the contact wrenches in the base frame */
		const rbd::BodyWrench& getContactWrench_B() const;
		const rbd::Vector6d& getContactWrench_B(std::string name) const;

		/**
		 * @brief Gets the contact condition (active or inactive)
		 * @param std::string Name of contact link
		 * @param double Force threshold for detecting contact condition
		 * @return bool True for active contacts, false for inactive ones
		 */
		bool getContactCondition(std::string name,
								 double force_threshold) const;
		bool getContactCondition(std::string name) const;

		// Contact state setter functions
		/** @brief Sets the contact positions in the base frame */
		void setContactPosition_B(const rbd::BodyVector& pos);
		void setContactPosition_B(std::string name,
								 const Eigen::VectorXd& pos);

		/** @brief Sets the contact velocities in the base frame */
		void setContactVelocity_B(const rbd::BodyVector& vel);
		void setContactVelocity_B(std::string name,
								  const Eigen::VectorXd& vel);

		/** @brief Sets the contact accelerations in the base frame */
		void setContactAcceleration_B(const rbd::BodyVector& acc);
		void setContactAcceleration_B(std::string name,
									  const Eigen::VectorXd& acc);

		/** @brief Sets the contact wrenches in the base frame */
		void setContactWrench_B(const rbd::BodyWrench& eff);
		void setContactWrench_B(std::string name,
							    const rbd::Vector6d& eff);

		/**
		 * @brief Sets the contact condition (active or inactive)
		 * @param std::string Name of the contact link
		 * @bool True for active conditions, and false for inactive
		 */
		void setContactCondition(std::string name,
								 bool condition);

		/** @brief Internal whole-body state variables expressed with the
		 * above mentioned convention */
		double time;
		double duration;
		rbd::Vector6d base_pos;
		rbd::Vector6d base_vel;
		rbd::Vector6d base_acc;
		rbd::Vector6d base_eff;
		Eigen::VectorXd joint_pos;
		Eigen::VectorXd joint_vel;
		Eigen::VectorXd joint_acc;
		Eigen::VectorXd joint_eff;
		rbd::BodyVector contact_pos;
		rbd::BodyVector contact_vel;
		rbd::BodyVector contact_acc;
		rbd::BodyWrench contact_eff;


	private:
		/** @brief Number of joints */
		unsigned int num_joints_;

		/** @brief Frame transformations */
		math::FrameTF frame_tf_;
};

/** @brief Defines a whole-body trajectory */
typedef std::vector<WholeBodyState> WholeBodyTrajectory;

} //@namespace dwl

#endif
