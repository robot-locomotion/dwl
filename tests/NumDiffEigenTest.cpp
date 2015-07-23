#include <model/FullDynamicalSystem.h>
#include <model/HS071DynamicalSystem.cpp>
#include <model/OptimizationModel.h>
#include <unsupported/Eigen/NumericalDiff>
#include <iostream>


dwl::model::OptimizationModel opt_model;



unsigned int n;// = 2;
unsigned int m;// = 9;

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

struct my_functor : Functor<double>
{
	my_functor(void): Functor<double>(n,m) {}
	int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
	{
		dwl::LocomotionState state;
		opt_model.getDynamicalSystem()->toLocomotionState(state, x);

		std::cout << "State = " << x.transpose() << std::endl;
		std::cout << "Base pos = " << state.base_pos.transpose() << std::endl;
		std::cout << "Joint pos = " << state.joint_pos.transpose() << std::endl;
		std::cout << "Base vel = " << state.base_vel.transpose() << std::endl;
		std::cout << "Joint vel = " << state.joint_vel.transpose() << std::endl;
		std::cout << "Base acc = " << state.base_acc.transpose() << std::endl;
		std::cout << "Joint acc = " << state.joint_acc.transpose() << std::endl;


//    	dynamical_system.compute(fvec, state);

		fvec.resize(2);
		opt_model.evaluateConstraints(fvec, x);


//    	std::cout << x.transpose() << " = state" << std::endl;
//    	std::cout << fvec.transpose() << " = constraint" << std::endl;
		std::cout << "----------------------------------------------------" << std::endl;

		return 0;
	}
};


struct ConstraintFunction : Functor<double>
{
	ConstraintFunction(void) : Functor<double>(0,0) {}
    int operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
    {
    	fvec.resize(2);
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


	// Initializating the robot model
	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	dwl::rbd::FloatingBaseSystem system;
	system.LZ.active = true;
	system.LZ.id = 0;
	system.setTypeOfDynamicSystem(dwl::rbd::VirtualFloatingBase);

	dwl::model::DynamicalSystem* dynamical_system = new dwl::model::FullDynamicalSystem();
	dynamical_system->setFloatingBaseSystem(&system);
	dynamical_system->getDynamics().modelFromURDFFile(model_file, &system, true);

	opt_model.addDynamicalSystem(dynamical_system);

	// Converting locomotion state to generalized coordinates
	Eigen::VectorXd q(3), qd(3), qdd(3);
	q = dwl::rbd::toGeneralizedJointState(base_pos, joint_pos, &system);
	qd = dwl::rbd::toGeneralizedJointState(base_vel, joint_vel, &system);
	qdd = dwl::rbd::toGeneralizedJointState(base_acc, joint_acc, &system);


	// Setting the state
	Eigen::VectorXd x(9);
	x << q, qd, qdd;
	Eigen::MatrixXd jac(2,9);


	n = 0;
	m = 0;

//	my_functor functor;
//	Eigen::NumericalDiff<my_functor,Eigen::Central> numDiff(functor, 1E-06);//2.2E-16
//	numDiff.df(x, jac);
//
//	std::cout << jac << std::endl;






//	struct ConstraintFunction : Functor<double>
//	{
//		ConstraintFunction(void) : Functor<double>(0,0) {}
//	    int operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
//	    {
//	    	fvec.resize(2);
//	    	opt_model.evaluateConstraints(fvec, x);
//	    	return 0;
//	    }
//	};



	ConstraintFunction functor;
	Eigen::NumericalDiff<ConstraintFunction,Eigen::Central> numDiff(functor, 1E-06);//2.2E-16
	numDiff.df(x, jac);
	std::cout << jac << std::endl;
}

