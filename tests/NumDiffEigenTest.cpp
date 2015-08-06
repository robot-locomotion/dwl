#include <model/FullDynamicalSystem.h>
#include <model/ConstrainedDynamicalSystem.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/OptimizationModel.h>
#include <unsupported/Eigen/NumericalDiff>
#include <iostream>


dwl::model::OptimizationModel opt_model;

template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
	typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
	typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }
};


struct ConstraintFunction : Functor<double>
{
	ConstraintFunction(void) : Functor<double>(0,0) {}
    int operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
    {
    	fvec.resize(5);
    	opt_model.evaluateConstraints(fvec, x);
    	return 0;
    }
};



int main(int argc, char **argv)
{
	// Defining a state (q, q_dot, q_ddot)
	dwl::rbd::Vector6d base_pos = dwl::rbd::Vector6d::Zero();
	dwl::rbd::Vector6d base_vel = dwl::rbd::Vector6d::Zero();
	dwl::rbd::Vector6d base_acc = dwl::rbd::Vector6d::Zero();
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(2);
	Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(2);
	Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(2);

	joint_pos(0) = 0.5;
	joint_pos(1) = -1.5;


	// Initialization the robot model
	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	dwl::rbd::FloatingBaseSystem system;
	system.LZ.active = true;
	system.LZ.id = 0;

	dwl::model::ConstrainedDynamicalSystem* constrained_system = new dwl::model::ConstrainedDynamicalSystem();
	dwl::rbd::BodySelector active_contact;
	active_contact.push_back("foot");
	constrained_system->setActiveEndEffectors(active_contact);
	dwl::model::DynamicalSystem* dynamical_system = constrained_system;
//		new dwl::model::FullDynamicalSystem();
//		new dwl::model::HS071DynamicalSystem();
	dynamical_system->modelFromURDFFile(model_file, true);

	opt_model.addDynamicalSystem(dynamical_system);

	// Converting locomotion state to generalized coordinates
	Eigen::VectorXd q(3), qd(3), qdd(3), tau(2);
	q = dwl::rbd::toGeneralizedJointState(base_pos, joint_pos, system);
	qd = dwl::rbd::toGeneralizedJointState(base_vel, joint_vel, system);
	qdd = dwl::rbd::toGeneralizedJointState(base_acc, joint_acc, system);
	tau = Eigen::VectorXd::Zero(2);


	// Setting the state
	Eigen::VectorXd x(8);
	x << q, qd, tau;
	Eigen::MatrixXd jac(5,8);


	ConstraintFunction functor;
	Eigen::NumericalDiff<ConstraintFunction,Eigen::Central> numDiff(functor, 1E-06);//2.2E-16
//	ConstraintFunction<2,9> functor;
//	Eigen::NumericalDiff<ConstraintFunction<2,9> ,Eigen::Central> numDiff(functor, 1E-05);//2.2E-16
	numDiff.df(x, jac);
	std::cout << jac << std::endl;
}

