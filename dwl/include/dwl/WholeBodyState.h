#ifndef DWL__WHOLE_BODY_STATE__H
#define DWL__WHOLE_BODY_STATE__H

#include <map>
#include <vector>

#include <Eigen/Dense>
#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

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

		/** @brief Gets the number of joints */
		const unsigned int getJointDof() const;


        /* setters */
//        void setPosition_W(const Eigen::Vector3d& pos);
//        void setPosition_W(const double& x,
//                           const double& y,
//                           const double& z);
//
//        void setVelocity_W(const Eigen::Vector3d& vel);
//        void setVelocity_W(const double& x,
//                              const double& y,
//                              const double& z);
//
//        void setVelocity_B(const Eigen::Vector3d& vel);
//        void setVelocity_B(const double& x,
//                           const double& y,
//                           const double& z);
//
//        void setVelocity_H(const Eigen::Vector3d& vel);
//        void setVelocity_H(const double& x,
//                           const double& y,
//                           const double& z);
//
//        void setAcceleration_W(const Eigen::Vector3d& acc);
//        void setAcceleration_W(const double& x,
//                               const double& y,
//                               const double& z);
//
//        void setAcceleration_I(const Eigen::Vector3d& acc,
//                               const Eigen::Affine3d& bTi);
//
//        void setAcceleration_I(const double& x,
//                               const double& y,
//                               const double& z,
//                               const Eigen::Affine3d& bTi);
//
//        void setAcceleration_B(const Eigen::Vector3d& acc);
//        void setAcceleration_B(const double& x,
//                               const double& y,
//                               const double& z);
//
//        void setRotationRate_W(const Eigen::Vector3d& rate);
//
//        void setRotAcceleration_W(const Eigen::Vector3d& rotacc);
//
//        void setRotationRate_B(const Eigen::Vector3d& rate);
//        void setRotationRate_B(const double& x,
//                               const double& y,
//                               const double& z);
//
//        void setRotAcceleration_B(const Eigen::Vector3d& rate);
//        void setRotAcceleration_B(const double& x,
//                                  const double& y,
//                                  const double& z);
//
//        void setOrientation_W(const Eigen::Quaterniond& orient);


		/**
		 * @brief Sets the number of joints
		 * @param unsigned int Number of joints
		 */
		void setJointDoF(unsigned int num_joints);



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
};

/** @brief Defines a whole-body trajectory */
typedef std::vector<WholeBodyState> WholeBodyTrajectory;

} //@namespace dwl

#endif
