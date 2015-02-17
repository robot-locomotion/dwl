#ifndef DWL_FloatingBaseKinematics_H
#define DWL_FloatingBaseKinematics_H

#include <Eigen/Dense>
#include <map>
#include <utils/utils.h>
#include <iit/rbd/rbd.h>


namespace dwl
{

namespace model
{

using namespace iit;

enum Component {Linear, Angular, Full};
typedef std::map<std::string,bool> ActiveContact;

class FloatingBaseKinematics
{
	public:
		FloatingBaseKinematics();
		virtual ~FloatingBaseKinematics();

		virtual void init() = 0;

		virtual void updateJointState(Eigen::VectorXd position, Eigen::VectorXd velocity) = 0;

		virtual void computeFloatingBaseJacobian(Eigen::MatrixXd& jacobian);
		virtual void computeFloatingBaseJacobian(Eigen::MatrixXd& jacobian, std::map<std::string,bool> active_contact);

		virtual void computeEffectorJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);
		virtual void computeEffectorJacobian(Eigen::MatrixXd& jacobian, std::map<std::string,bool> active_contact,
				enum Component component = Full);

		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);
		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian, std::map<std::string,bool> active_contact,
						enum Component component = Full);

		typedef std::map<int,std::string> EndEffectorID;
		typedef std::map<std::string, Eigen::Matrix<double, 6, Eigen::Dynamic> > EndEffectorJacobian;



	protected:
		EndEffectorID effector_id_;
		EndEffectorJacobian current_jacs_;

		int num_joints_;
		int floating_base_dof_;
		Eigen::MatrixXd floating_base_rot_;


	private:

};

} //@namespace model
} //@namespace dwl

#endif
