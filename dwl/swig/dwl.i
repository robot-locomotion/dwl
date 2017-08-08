%module dwl
%{
#include <Python.h>
#include <dwl/ReducedBodyState.h>
#include <dwl/WholeBodyState.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/RobotStates.h>
%}


/// Data structure in the target language holding data
#ifdef SWIGPYTHON
#define GUESTOBJECT PyObject
#elif defined(SWIGMATLAB)
#define GUESTOBJECT mxArray
#else
#define GUESTOBJECT void
#endif

// typemaps.i is a built-in swig interface that lets us map c++ types to other
// types in our language of choice. We'll use it to map Eigen matrices to
// Numpy arrays.
%include <typemaps.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_map.i>

// eigen.i is found in ../swig/ and contains specific definitions to convert
// Eigen matrices into Numpy arrays.
%include <eigen.i>


%template(Matrix3d_List) std::vector<Eigen::Matrix3d>;
%template(Matrix4d_List) std::vector<Eigen::Matrix4d>;
%template(MatrixXd_List) std::vector<Eigen::MatrixXd>;
%template(Vector2d_List) std::vector<Eigen::Vector2d>;
%template(Vector3d_List) std::vector<Eigen::Vector3d>;
%template(VectorXd_List) std::vector<Eigen::VectorXd>;
//%template(Vector3d_Dict) std::map<std::string,Eigen::Vector3d>;
//%template(VectorXd_Dict) std::map<std::string,Eigen::VectorXd>;

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(dwl::rbd::Vector6d)
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::MatrixXd)
//%eigen_typemaps(Eigen::Quaterniond) TODO it doesn't work yet
// Even though Eigen::MatrixXd is just a typedef for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>, our templatedInverse function doesn't
// compile correctly unless we also declare typemaps for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>. Not totally sure why that is.
//%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

%template(string_List) std::vector<std::string>;
%template(string_uint) std::map<std::string,unsigned int>;
%template(string_jointLimits) std::map<std::string,urdf::JointLimits>;


// Renaming orientation methods to get rid of the ambiguity
%rename(getQuaternion_RM) 
		getQuaternion(const Eigen::Matrix3d&);
%rename(getQuaternion_RPY) 
		getQuaternion(const Eigen::Vector3d&);
%rename(getInverseEulerAnglesRatesMatrix_RPY)
		getInverseEulerAnglesRatesMatrix(const Eigen::Vector3d&);
%rename(getInverseEulerAnglesRatesMatrix_RM)
		getInverseEulerAnglesRatesMatrix(const Eigen::Matrix3d&);
%rename(getInverseEulerAnglesRatesMatrix_Q)
		getInverseEulerAnglesRatesMatrix(const Eigen::Quaterniond&);
%rename(getEulerAnglesRatesMatrix_RPY)
		getEulerAnglesRatesMatrix(const Eigen::Vector3d&);
%rename(getEulerAnglesRatesMatrix_RM)
		getEulerAnglesRatesMatrix(const Eigen::Matrix3d&);
%rename(getEulerAnglesRatesMatrix_Q)
		getEulerAnglesRatesMatrix(const Eigen::Quaterniond&);


// Renaming few functions of the WholeBodyState class to get rid of the ambiguity
%rename(setContactPositionDict_W) setContactPosition_W(const rbd::BodyVectorXd&);
%rename(setContactPositionDict_B) setContactPosition_B(const rbd::BodyVectorXd&);
%rename(setContactPositionDict_H) setContactPosition_H(const rbd::BodyVectorXd&);
%rename(setContactVelocityDict_W) setContactVelocity_W(const rbd::BodyVectorXd&);
%rename(setContactVelocityDict_B) setContactVelocity_B(const rbd::BodyVectorXd&);
%rename(setContactVelocityDict_H) setContactVelocity_H(const rbd::BodyVectorXd&);
%rename(setContactAccelerationDict_W) setContactAcceleration_W(const rbd::BodyVectorXd&);
%rename(setContactAccelerationDict_B) setContactAcceleration_B(const rbd::BodyVectorXd&);
%rename(setContactAccelerationDict_H) setContactAcceleration_H(const rbd::BodyVectorXd&);
%rename(setContactWrenchDict_B) setContactWrench_B(const rbd::BodyVector6d&);


// Renaming few functions of the ReducedBodyState class to get rid of the ambiguity
%rename(setFootPositionDict_W) setFoottPosition_W(const rbd::BodyVectorXd&);
%rename(setFootPositionDict_B) setFootPosition_B(const rbd::BodyVectorXd&);
%rename(setFootPositionDict_H) setFootPosition_H(const rbd::BodyVectorXd&);
%rename(setFootVelocityDict_W) setFootVelocity_W(const rbd::BodyVectorXd&);
%rename(setFootVelocityDict_B) setFootVelocity_B(const rbd::BodyVectorXd&);
%rename(setFootVelocityDict_H) setFootVelocity_H(const rbd::BodyVectorXd&);
%rename(setFootAccelerationDict_W) setFootAcceleration_W(const rbd::BodyVectorXd&);
%rename(setFootAccelerationDict_B) setFootAcceleration_B(const rbd::BodyVectorXd&);
%rename(setFootAccelerationDict_H) setFootAcceleration_H(const rbd::BodyVectorXd&);

// Ignoring two methods of the WholeBodyKinematic class that generate
// ambiguity
%ignore computeJointPosition(Eigen::VectorXd&,
							 const rbd::BodyVector3d&);
%ignore computeInverseKinematics(rbd::Vector6d&,
								 Eigen::VectorXd&,
								 const rbd::BodyVector3d&);


// Renaming some functions that generate ambiguity in the WholeBodyDynamic class
%rename(computeInverseDynamics_withoutFex)
		computeInverseDynamics(rbd::Vector6d&,
							   Eigen::VectorXd&,
							   const rbd::Vector6d&,
							   const Eigen::VectorXd&,
							   const rbd::Vector6d&,
							   const Eigen::VectorXd&,
							   const rbd::Vector6d&,
							   const Eigen::VectorXd&);
%rename(computeFloatingBaseInverseDynamics_withoutFex)
		computeFloatingBaseInverseDynamics(rbd::Vector6d&,
										   Eigen::VectorXd&,
										   const rbd::Vector6d&,
										   const Eigen::VectorXd&,
										   const rbd::Vector6d&,
										   const Eigen::VectorXd&,
										   const Eigen::VectorXd&);

%rename(urdf_Joint) urdf::Joint;
%rename(urdf_Pose) urdf::Pose;
%include <dwl/utils/RigidBodyDynamics.h>
%include <dwl/ReducedBodyState.h>
%include <dwl/WholeBodyState.h>
%include <urdf_model/pose.h>
%include <urdf_model/joint.h>
%include <dwl/utils/URDF.h>
%include <dwl/utils/Orientation.h>
%include <dwl/model/FloatingBaseSystem.h>
%include <dwl/model/WholeBodyKinematics.h>
%include <dwl/model/WholeBodyDynamics.h>
%include <dwl/RobotStates.h>
%extend dwl::ReducedBodyState {
	const char *__str__() {
		std::stringstream buffer;
		buffer << "ReducedBodyState:" << std::endl;
		buffer << "	com_pos: " << $self->getCoMPosition_W().transpose() << std::endl;
		buffer << "	com_vel: " << $self->getCoMVelocity_W().transpose() << std::endl;
		buffer << "	com_acc: " << $self->getCoMAcceleration_W().transpose() << std::endl;
		buffer << "	angular_pos: " << $self->getRPY_W().transpose() << std::endl;
		buffer << "	angular_vel: " << $self->getAngularVelocity_W().transpose() << std::endl;
		buffer << "	angular_acc: " << $self->getAngularAcceleration_W().transpose() << std::endl;
		buffer << "	cop: " << $self->getCoPPosition_W().transpose() << std::endl;
		buffer << "	foot_pos: " << std::endl;
		for (dwl::rbd::BodyVector3d::const_iterator it = $self->getFootPosition_B().begin();
				it != $self->getFootPosition_B().end(); ++it) {
			buffer << "		" << it->first << ": " << it->second.transpose() << std::endl;
		}
		buffer << "	foot_vel: " << std::endl;
		for (dwl::rbd::BodyVector3d::const_iterator it = $self->getFootVelocity_B().begin();
				it != $self->getFootVelocity_B().end(); ++it) {
			buffer << "		" << it->first << ": " << it->second.transpose() << std::endl;
		}
		buffer << "	foot_acc: " << std::endl;
		for (dwl::rbd::BodyVector3d::const_iterator it = $self->getFootAcceleration_B().begin();
				it != $self->getFootAcceleration_B().end(); ++it) {
			buffer << "		" << it->first << ": " << it->second.transpose() << std::endl;
		}
		
		const std::string tmp = buffer.str();
		return tmp.c_str();
	}
};