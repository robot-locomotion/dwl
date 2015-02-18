#ifndef DWL_WholeBodyKinematics_H
#define DWL_WholeBodyKinematics_H

#include <Eigen/Dense>
#include <map>
#include <utils/utils.h>
#include <utils/Math.h>
#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>


namespace dwl
{

namespace model
{

using namespace iit;

enum Component {Linear, Angular, Full};


class WholeBodyKinematics
{
	public:
		WholeBodyKinematics();
		virtual ~WholeBodyKinematics();

		virtual void init() = 0;

		virtual void updateJointState(Eigen::VectorXd position, Eigen::VectorXd velocity) = 0;

		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);
		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian, EndEffectorSelector effector_set,
											  enum Component component = Full);

		virtual void computeBaseJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);
		virtual void computeBaseJacobian(Eigen::MatrixXd& jacobian, EndEffectorSelector effector_set,
										 enum Component component = Full);

		virtual void computeEffectorJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);
		virtual void computeEffectorJacobian(Eigen::MatrixXd& jacobian, EndEffectorSelector effector_set,
											 enum Component component = Full);

		typedef std::map<std::string, Eigen::Matrix<double, 6, Eigen::Dynamic> > EndEffectorJacobian;


	protected:
		EndEffectorID effector_id_;
		EndEffectorJacobian current_jacs_;
		EndEffectorPosition current_effector_pos_;

		int num_joints_;
		Eigen::MatrixXd floating_base_rot_;


	private:

};

} //@namespace model
} //@namespace dwl

#endif
