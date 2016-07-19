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


//		const Eigen::Vector3d& getPosition_W() const;
//        const Eigen::Vector3d& getRPY_W() const;
//        const Eigen::Vector3d& getRotationRate_B() const;
//        const Eigen::Quaterniond& getOrientation_W() const;
//        const Eigen::Quaterniond getOrientation_H() const;
//
//		const Eigen::Vector3d getVelocity_W() const;
//		const Eigen::Vector3d& getVelocity_B() const;
//        const Eigen::Vector3d getVelocity_H() const;
//
//        const Eigen::Vector3d& getAcceleration_B() const;
//        const Eigen::Vector3d& getRotAcceleration_B() const;






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

		/**
		 * @brief Gets the number of joints
		 * @return const unsigned int Number of joints
		 */
		const unsigned int getJointDof() const;


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
};

/** @brief Defines a whole-body trajectory */
typedef std::vector<WholeBodyState> WholeBodyTrajectory;

} //@namespace dwl

#endif
