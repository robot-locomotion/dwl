#ifndef DWL_WholeBodyDynamics_H
#define DWL_WholeBodyDynamics_H

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

		virtual void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
		        const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_accel,
		        const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd) = 0;
//		, const ExtForces& fext = zeroExtForces) = 0;

};

} //@namespace model
} //@namespace dwl

#endif
