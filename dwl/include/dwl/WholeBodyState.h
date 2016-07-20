#ifndef DWL__WHOLE_BODY_STATE__H
#define DWL__WHOLE_BODY_STATE__H

#include <map>
#include <vector>

#include <Eigen/Dense>
#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

/**
 * @brief The WholeBodyState class
 * This class incorporate the whole-body state of the robot, including:
 * <ul>
 *    <li>time, in seconds</li>
 *    <li>Base position, expressed in a fixed-world frame of reference</li>
 *    <li>Base velocity, expressed in the base frame</li>
 *    <li>Base acceleration, expressed in the base frame</li>
 *    <li>Base orientation, expressed in a fixed-world frame of reference</li>
 *    <li>Base rotation rate, expressed in the base frame</li>
 *    <li>Joint positions, expressed with the urdf order</li>
 *    <li>Joint velocities, expressed with the urdf order</li>
 *    <li>Joint accelerations, expressed with the urdf order</li>
 *    <li>Joint efforts, expressed with the urdf order</li>
 *    <li>Contact positions, expressed in the base frame</li>
 *    <li>Contact velocites, expressed in the base frame</li>
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
 * are provided. Both Eigen and base types are available for setter methods
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
		Eigen::Vector3d getPosition_W();

		/** @brief Gets the base orientation in the world frame */
		Eigen::Quaterniond getOrientation_W();

		/** @brief Gets the base RPY angles in the world frame */
		Eigen::Vector3d getRPY_W();

		/** @brief Gets the base orientation in the horizontal frame */
		Eigen::Quaterniond getOrientation_H();

		/** @brief Gets the base velocity in the world frame */
		Eigen::Vector3d getVelocity_W();

		/** @brief Gets the base velocity in the base frame */
		Eigen::Vector3d getVelocity_B();

		/** @brief Gets the base velocity in the horizontal frame */
		Eigen::Vector3d getVelocity_H();

		/** @brief Gets the base rotation rate in the base frame */
		Eigen::Vector3d getRotationRate_B();

		/** @brief Gets the base acceleration in the world frame */
		Eigen::Vector3d getAcceleration_W();

		/** @brief Gets the base acceleration in the base frame */
		Eigen::Vector3d getAcceleration_B();

		/** @brief Gets the base acceleration in the horizontal frame */
		Eigen::Vector3d getAcceleration_H();

		/** @brief Gets the base rotation acceleration in the base frame */
		Eigen::Vector3d getRotAcceleration_B();

		// Base state setter functions
		void setPosition_W(const Eigen::Vector3d& pos);
		void setOrientation_W(const Eigen::Quaterniond& orient);
		void setRPY_W(const Eigen::Vector3d& rpy);

		void setVelocity_W(const Eigen::Vector3d& vel); //TODO review
//		void setVelocity_B(const Eigen::Vector3d& vel);
//		void setVelocity_H(const Eigen::Vector3d& vel);
		void setRotationRate_W(const Eigen::Vector3d& rate); //TODO review
		void setRotationRate_B(const Eigen::Vector3d& rate); //TODO review

		void setAcceleration_W(const Eigen::Vector3d& acc); //TODO review
//        void setAcceleration_I(const Eigen::Vector3d& acc,
//                               const Eigen::Affine3d& bTi);
//		void setAcceleration_B(const Eigen::Vector3d& acc); //TODO review
		void setRotAcceleration_W(const Eigen::Vector3d& rotacc); //TODO review
		void setRotAcceleration_B(const Eigen::Vector3d& rate); //TODO review


		// Joint state getter functions
		/** @brief Gets the joint positions */
		const Eigen::VectorXd& getJointPosition() const;

		/** @brief Gets the joint velocities */
		const Eigen::VectorXd& getJointVelocity() const;

		/** @brief Gets the joint accelerations */
		const Eigen::VectorXd& getJointAcceleration() const;

		/** @brief Gets the joint effort */
		const Eigen::VectorXd& getJointEffort() const;

		/** @brief Gets the number of joints */
		const unsigned int getJointDof() const;

		// Joint state setter functions
		/** @brief Sets the joint positions */
		void setJointPosition(const Eigen::VectorXd& pos);

		/** @brief Sets the joint velocities */
		void setJointVelocity(const Eigen::VectorXd& vel);

		/** @brief Sets the joint accelerations */
		void setJointAcceleration(const Eigen::VectorXd& acc);

		/** @brief Sets the joint efforts */
		void setJointEffort(const Eigen::VectorXd& eff);

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

		/** @brief Gets the contact efforts in the base frame */
		const rbd::BodyWrench& getContactEffort_B() const;
		const rbd::Vector6d& getContactEffort_B(std::string name) const;

		/**
		 * @brief Gets the contact condition (active or inactive)
		 * @param std::string Name of contact link
		 * @param double Force threshold for detecting contact condition
		 * @return bool True for active contacts, false for inactive ones
		 */
		bool getContactCondition(std::string name,
								 double force_threshold) const;

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

		/** @brief Sets the contact efforts in the base frame */
		void setContactEffort_B(const rbd::BodyWrench& eff);
		void setContactEffort_B(std::string name,
							    const rbd::Vector6d& eff);

		/**
		 * @brief Sets the contact condition (active or inactive)
		 * @param std::string Name of the contact link
		 * @bool True for active conditions, and false for inactive
		 */
		void setContactCondition(std::string name,
								 bool condition);

//	private:
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
		Eigen::Matrix3d inline getRotBaseToHF() {
			Eigen::Matrix3d R;
			Eigen::Vector3d rpy_W = getRPY_W();

			R <<  cos(rpy_W(1)),  sin(rpy_W(0))*sin(rpy_W(1)),  cos(rpy_W(0))*sin(rpy_W(1)),
					         0.,                cos(rpy_W(0)),               -sin(rpy_W(0)),
				 -sin(rpy_W(1)),  sin(rpy_W(0))*cos(rpy_W(1)),  cos(rpy_W(0))*cos(rpy_W(1));
			return R;
		}

		/** @brief Number of joints */
		unsigned int num_joints_;
};

/** @brief Defines a whole-body trajectory */
typedef std::vector<WholeBodyState> WholeBodyTrajectory;

} //@namespace dwl

#endif
