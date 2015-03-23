#ifndef DWL_WholeBodyDynamics_H
#define DWL_WholeBodyDynamics_H

#include <model/WholeBodyKinematics.h>
#include <iit/rbd/rbd.h>


namespace dwl
{

namespace model
{


class WholeBodyDynamics
{
	public:
		WholeBodyDynamics();
		virtual ~WholeBodyDynamics();

		void setKinematicModel(model::WholeBodyKinematics* kin_model_);

		virtual void init() = 0;
		virtual void updateState(const iit::rbd::Vector6D& base_pos, const Eigen::VectorXd& joint_pos) = 0;

		virtual void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
																	const iit::rbd::Vector6D& base_pos,
																	const iit::rbd::Vector6D& base_vel,
																	const Eigen::VectorXd& joint_pos,
																	const Eigen::VectorXd& joint_vel) = 0;
		virtual void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
																	EndEffectorSelector effector_set,
																	const iit::rbd::Vector6D& base_pos,
																	const iit::rbd::Vector6D& base_vel,
																	const Eigen::VectorXd& joint_pos,
																	const Eigen::VectorXd& joint_vel) = 0;

		virtual void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
		        									 const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_pos,
		        									 const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_acc,
		        									 const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& joint_vel,
		        									 const Eigen::VectorXd& joint_acc) = 0;
//		, const ExtForces& fext = zeroExtForces) = 0;

		typedef std::map<std::string, Eigen::Matrix<double, 6, 6> > EndEffectorSpatialTransform;
		typedef std::map<std::string, Eigen::Matrix<double, 6, 1> > EndEffectorSpatialVector;


	protected:
		/** @brief End-effector ids */
		EndEffectorID effector_id_;

		EndEffectorSpatialTransform closest_link_motion_tf_;
		EndEffectorSpatialVector closest_link_velocity_;
		EndEffectorSpatialVector closest_link_acceleration_;
		model::WholeBodyKinematics* kin_model_;

};

} //@namespace model
} //@namespace dwl

#endif
