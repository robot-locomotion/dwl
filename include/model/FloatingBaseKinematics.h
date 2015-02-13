#ifndef DWL_FloatingBaseKinematics_H
#define DWL_FloatingBaseKinematics_H



namespace dwl
{

namespace model
{

int num_joints_;

class FloatingBaseKinematics
{
	public:
		FloatingBaseKinematics();
		virtual ~FloatingBaseKinematics();

		virtual void computeBaseJacobian(Eigen::Matrix<double, Eigen::Dynamic, num_joints_>& jacobian) = 0;
		virtual void computeEndEffectorJacobian(Eigen::Matrix<double, Eigen::Dynamic, num_joints_>& jacobian) = 0;


	private:
};

} //@namespace model
} //@namespace dwl

#endif
