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
		virtual void updateState(Eigen::VectorXd state) = 0;

		virtual void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
				Eigen::VectorXd q, Eigen::VectorXd qd) = 0;
		virtual void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
				EndEffectorSelector effector_set, Eigen::VectorXd q, Eigen::VectorXd qd) = 0;

		virtual void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
		        const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_accel,
		        const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd) = 0;
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
