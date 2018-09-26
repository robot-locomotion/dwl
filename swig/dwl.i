%module(directors="1") dwl
%{
#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS 
#define SWIG_FILE_WITH_INIT
#include <Python.h>

// Robot-related core functions
#include <dwl/ReducedBodyState.h>
#include <dwl/WholeBodyState.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/WholeBodyDynamics.h>
#include <dwl/RobotStates.h>

// Optimization-related core functions
#include <dwl/model/OptimizationModel.h>
#include <dwl/solver/OptimizationSolver.h>
#include <dwl/solver/IpoptNLP.h>
#include <dwl/solver/cmaesSOFamily.h>

// Yaml parser
#include <dwl/utils/YamlWrapper.h>
%}


// typemaps.i is a built-in swig interface that lets us map c++ types to other
// types in our language of choice. We'll use it to map Eigen matrices to
// Numpy arrays.
%include <typemaps.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_map.i>



////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Robot-related core functions ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// eigen.i is found in ../swig/ and contains specific definitions to convert
// Eigen matrices into Numpy arrays.
%include <eigen.i>


%template(Vector2dList) std::vector<Eigen::Vector2d>;
%template(Vector3dList) std::vector<Eigen::Vector3d>;
%template(Vector4dList) std::vector<Eigen::Vector4d>;
%template(Vector6dList) std::vector<Eigen::Vector6d>;
%template(Vector7dList) std::vector<Eigen::Vector7d>;
%template(VectorXdList) std::vector<Eigen::VectorXd>;
%template(Matrix3dList) std::vector<Eigen::Matrix3d>;
%template(Matrix4dList) std::vector<Eigen::Matrix4d>;
%template(Matrix6dList) std::vector<Eigen::Matrix6d>;
%template(Matrix6xList) std::vector<Eigen::Matrix6x>;
%template(MatrixXdList) std::vector<Eigen::MatrixXd>;

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Vector4d)
%eigen_typemaps(Eigen::Vector6d)
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix6d)
%eigen_typemaps(Eigen::Matrix6x)
//%eigen_typemaps(Eigen::Quaterniond) TODO it doesn't work yet
// Even though Eigen::MatrixXd is just a typedef for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>, our templatedInverse function doesn't
// compile correctly unless we also declare typemaps for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>. Not totally sure why that is.
//%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

%template(doubleList) std::vector<double>;
%template(stringList) std::vector<std::string>;
%template(uintMap) std::map<std::string,unsigned int>;
%template(JointLimitsMap) std::map<std::string,urdf::JointLimits>;
%template(SE3Map) std::map<std::string,dwl::SE3>;
%template(MotionMap) std::map<std::string,dwl::Motion>;
%template(ForceMap) std::map<std::string,dwl::Force>;


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


// Renaming few functions of the Lie group to get rid of the ambiguity
%rename(SE3_RPY) 
		SE3(const Eigen::Vector3d&, const Eigen::Vector3d&);
%rename(SE3_Q) 
		SE3(const Eigen::Vector3d&, const Eigen::Vector4d&);


// Renaming few functions of the ReducedBodyState class to get rid of the ambiguity
%rename(setFootPositionDict_W) setFootPosition_W(const rbd::BodyVectorXd&);
%rename(setFootPositionDict_B) setFootPosition_B(const rbd::BodyVectorXd&);
%rename(setFootPositionDict_H) setFootPosition_H(const rbd::BodyVectorXd&);
%rename(setFootVelocityDict_W) setFootVelocity_W(const rbd::BodyVectorXd&);
%rename(setFootVelocityDict_B) setFootVelocity_B(const rbd::BodyVectorXd&);
%rename(setFootVelocityDict_H) setFootVelocity_H(const rbd::BodyVectorXd&);
%rename(setFootAccelerationDict_W) setFootAcceleration_W(const rbd::BodyVectorXd&);
%rename(setFootAccelerationDict_B) setFootAcceleration_B(const rbd::BodyVectorXd&);
%rename(setFootAccelerationDict_H) setFootAcceleration_H(const rbd::BodyVectorXd&);


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
%include <dwl/utils/LieGroup.h>
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

// Extending the C++ class by adding printing methods in python
%extend std::vector<std::string> {
	char *__repr__() {
		std::stringstream buffer;
		buffer << "['";
		for (unsigned int i = 0; i < $self->size(); ++i) {
			if (i == $self->size() - 1)
				buffer << $self->at(i) << "']" << std::endl;
			else
				buffer << $self->at(i) << "', '";
		}
		std::string str = buffer.str();
		char * writable = new char[str.size() + 1];
		std::copy(str.begin(), str.end(), writable);
		writable[str.size()] = '\0';

		return writable;
	}
}
%extend dwl::SE3 {
	char *__repr__() {
		std::stringstream buffer;
		buffer << "\tt: " << $self->getTranslation().transpose() << std::endl;
		buffer << "\tR: " << $self->getRotation().row(0) << std::endl;
		buffer << "\t   " << $self->getRotation().row(1) << std::endl;
		buffer << "\t   " << $self->getRotation().row(2);
		std::string str = buffer.str();
		char * writable = new char[str.size() + 1];
		std::copy(str.begin(), str.end(), writable);
		writable[str.size()] = '\0';

		return writable;
	}	
}
%extend dwl::Motion {
	char *__repr__() {
		std::stringstream buffer;
		buffer << "\tv: " << $self->getLinear().transpose() << std::endl;
		buffer << "\tw: " << $self->getAngular().transpose();
		std::string str = buffer.str();
		char * writable = new char[str.size() + 1];
		std::copy(str.begin(), str.end(), writable);
		writable[str.size()] = '\0';

		return writable;
	}
}
%extend dwl::Force {
	char *__repr__() {
		std::stringstream buffer;
		buffer << "\tf: " << $self->getLinear().transpose() << std::endl;
		buffer << "\tn: " << $self->getAngular().transpose();
		std::string str = buffer.str();
		char * writable = new char[str.size() + 1];
		std::copy(str.begin(), str.end(), writable);
		writable[str.size()] = '\0';

		return writable;
	}
}
%extend dwl::ReducedBodyState {
	char *__repr__() {
		std::stringstream buffer;
		buffer << "ReducedBodyState:" << std::endl;
		buffer << "\ttime: " << $self->time << std::endl;
		buffer << "\tcom_pos:" << std::endl;
		buffer << "\t\tt: " << $self->getCoMSE3().getTranslation().transpose() << std::endl;
		buffer << "\t\tR: " << $self->getCoMSE3().getRotation().row(0) << std::endl;
		buffer << "\t\t   " << $self->getCoMSE3().getRotation().row(1) << std::endl;
		buffer << "\t\t   " << $self->getCoMSE3().getRotation().row(2) << std::endl;
		buffer << "\tcom_vel:" << std::endl;
		buffer << "\t\tv: " << $self->getCoMVelocity_B().getLinear().transpose() << std::endl;
		buffer << "\t\tw: " << $self->getCoMVelocity_B().getAngular().transpose() << std::endl;
		buffer << "\tcom_acc:" << std::endl;
		buffer << "\t\tvd: " << $self->getCoMAcceleration_B().getLinear().transpose() << std::endl;
		buffer << "\t\twd: " << $self->getCoMAcceleration_B().getAngular().transpose() << std::endl;
		buffer << "\tcop: " << $self->getCoPPosition_W().transpose() << std::endl;
		buffer << "\tfoot_pos_B: " << std::endl;
		for (dwl::SE3Map::const_iterator it = $self->getFootSE3_B().begin();
				it != $self->getFootSE3_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tt: " << it->second.getTranslation().transpose() << std::endl;
			buffer << "\t\t\tR: " << it->second.getRotation().row(0) << std::endl;
			buffer << "\t\t\t   " << it->second.getRotation().row(1) << std::endl;
			buffer << "\t\t\t   " << it->second.getRotation().row(2) << std::endl;
		}
		buffer << "\tfoot_vel_B: " << std::endl;
		for (dwl::MotionMap::const_iterator it = $self->getFootVelocity_B().begin();
				it != $self->getFootVelocity_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tv: " << it->second.getLinear().transpose() << std::endl;
			buffer << "\t\t\tw: " << it->second.getAngular().transpose() << std::endl;
		}
		buffer << "\tfoot_acc_B: " << std::endl;
		for (dwl::MotionMap::const_iterator it = $self->getFootAcceleration_B().begin();
				it != $self->getFootAcceleration_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tv: " << it->second.getLinear().transpose() << std::endl;
			buffer << "\t\t\tw: " << it->second.getAngular().transpose() << std::endl;
		}
		buffer << "	support_region: " << std::endl;
		for (dwl::SE3Map::const_iterator it = $self->support_region.begin();
				it != $self->support_region.end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tt: " << it->second.getTranslation().transpose() << std::endl;
			buffer << "\t\t\tR: " << it->second.getRotation().row(0) << std::endl;
			buffer << "\t\t\t   " << it->second.getRotation().row(1) << std::endl;
			buffer << "\t\t\t   " << it->second.getRotation().row(2) << std::endl;
		}
		std::string str = buffer.str();
		char * writable = new char[str.size() + 1];
		std::copy(str.begin(), str.end(), writable);
		writable[str.size()] = '\0';

		return writable;
	}
};
%extend dwl::WholeBodyState {
	char *__repr__() {
		std::stringstream buffer;
		buffer << "WholeBodyState:" << std::endl;
		buffer << "\ttime: " << $self->time << std::endl;
		buffer << "\tbase_pos:" << std::endl;
		buffer << "\t\tt: " << $self->getBaseSE3().getTranslation().transpose() << std::endl;
		buffer << "\t\tR: " << $self->getBaseSE3().getRotation().row(0) << std::endl;
		buffer << "\t\t   " << $self->getBaseSE3().getRotation().row(1) << std::endl;
		buffer << "\t\t   " << $self->getBaseSE3().getRotation().row(2) << std::endl;
		buffer << "\tbase_vel:" << std::endl;
		buffer << "\t\tv: " << $self->getBaseVelocity_B().getLinear().transpose() << std::endl;
		buffer << "\t\tw: " << $self->getBaseVelocity_B().getAngular().transpose() << std::endl;
		buffer << "\tbase_acc:" << std::endl;
		buffer << "\t\tvd: " << $self->getBaseAcceleration_B().getLinear().transpose() << std::endl;
		buffer << "\t\twd: " << $self->getBaseAcceleration_B().getAngular().transpose() << std::endl;
		buffer << "\tjoint_pos: " << $self->getJointPosition().transpose() << std::endl;
		buffer << "\tjoint_vel: " << $self->getJointVelocity().transpose() << std::endl;
		buffer << "\tjoint_acc: " << $self->getJointAcceleration().transpose() << std::endl;
		buffer << "\tjoint_eff: " << $self->getJointEffort().transpose() << std::endl;
		buffer << "\tcontact_pos_B: " << std::endl;
		for (dwl::SE3Map::const_iterator it = $self->getContactSE3_B().begin();
				it != $self->getContactSE3_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tt: " << it->second.getTranslation().transpose() << std::endl;
			buffer << "\t\t\tR: " << it->second.getRotation().row(0) << std::endl;
			buffer << "\t\t\t   " << it->second.getRotation().row(1) << std::endl;
			buffer << "\t\t\t   " << it->second.getRotation().row(2) << std::endl;
		}
		buffer << "\tcontact_vel_B: " << std::endl;
		for (dwl::MotionMap::const_iterator it = $self->getContactVelocity_B().begin();
				it != $self->getContactVelocity_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tv: " << it->second.getLinear().transpose() << std::endl;
			buffer << "\t\t\tw: " << it->second.getAngular().transpose() << std::endl;
		}
		buffer << "\tcontact_acc_B: " << std::endl;
		for (dwl::MotionMap::const_iterator it = $self->getContactAcceleration_B().begin();
				it != $self->getContactAcceleration_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tv: " << it->second.getLinear().transpose() << std::endl;
			buffer << "\t\t\tw: " << it->second.getAngular().transpose() << std::endl;
		}
		buffer << "\tcontact_eff_B: " << std::endl;
		for (dwl::ForceMap::const_iterator it = $self->getContactWrench_B().begin();
				it != $self->getContactWrench_B().end(); ++it) {
			buffer << "\t\t" << it->first << ":" << std::endl;
			buffer << "\t\t\tv: " << it->second.getLinear().transpose() << std::endl;
			buffer << "\t\t\tw: " << it->second.getAngular().transpose() << std::endl;
		}
		std::string str = buffer.str();
		char * writable = new char[str.size() + 1];
		std::copy(str.begin(), str.end(), writable);
		writable[str.size()] = '\0';

		return writable;
	}
};




////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Optimization-related core functions ////////////////////////
////////////////////////////////////////////////////////////////////////////////////
%feature("director") dwl::model::OptimizationModel;
%feature("director:except") {
	if( $error != NULL ) {
		PyObject *ptype, *pvalue, *ptraceback;
		PyErr_Fetch( &ptype, &pvalue, &ptraceback );
		PyErr_Restore( ptype, pvalue, ptraceback );
		PyErr_Print();
		Py_Exit(1);
	}
}

%include "numpy.i"
%init %{
	import_array();
%}
// For the typemap of the optimization model interface
%apply double& INOUT { double& cost };
%apply (double* IN_ARRAY1, int DIM1) {(double* decision, int decision_dim),
									  (const double* decision, int decision_dim),
									  (double* decision_lbound, int decision_dim1),
									  (double* decision_ubound, int decision_dim2),
									  (double* constraint_lbound, int constraint_dim1),
									  (double* constraint_ubound, int constraint_dim2),
									  (double* gradient, int grad_dim),
									  (double* constraint, int constraint_dim),
									  (double* jacobian_values, int nonzero_dim1),
									  (double* hessian_values, int nonzero_dim1),
									  (const double* lagrange, int constraint_dim)}
%apply (int* IN_ARRAY1, int DIM1) {(int* row_entries, int nonzero_dim2),
								   (int* col_entries, int nonzero_dim3)}
%typemap(directorin,numinputs=1) double &
{
	npy_intp dim = 1;
	$input = PyArray_SimpleNewFromData(1, &dim, NPY_DOUBLE, (void *)&$1);
}

%include <dwl/model/OptimizationModel.h>
%include <dwl/solver/OptimizationSolver.h>
%include <dwl/solver/IpoptNLP.h>
%include <dwl/solver/cmaesSOFamily.h>
%template(cmaesSO) dwl::solver::cmaesSOFamily<>;



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Yaml parser functions ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
%apply bool &OUTPUT { bool& data };
%apply int &OUTPUT { int& data };
%apply double &OUTPUT { double& data };
%apply std::string &OUTPUT { std::string& data };
%apply std::vector<double> &OUTPUT { std::vector<double>& data };
%apply std::vector<std::string> &OUTPUT { std::vector<std::string>& data };
%ignore read(bool &,std::string const &);
%rename(readBool)
		read(bool& data,
			 const std::string&,
			 const YamlNamespace& ns);
%ignore read(int &,std::string const &);
%ignore read(int &,std::string const &,YAML::Node const &);
%rename(readInt)
		read(int& data,
			 const std::string& field,
			 const YamlNamespace& ns);
%ignore read(double &,std::string const &);
%ignore read(double &,std::string const &,YAML::Node const &);
%rename(readDouble)
		read(double& data,
			 const std::string& field,
			 const YamlNamespace& ns);
%ignore read(std::string &,std::string const &);
%ignore read(std::string &,std::string const &,YAML::Node const &);
%rename(readString)
		read(std::string& data,
			 const std::string& field,
			 const YamlNamespace& ns);
%ignore read(std::vector<double> &,std::string const &);
%ignore read(std::vector<double> &,std::string const &,YAML::Node const &);
%rename(readDoubleList)
		read(std::vector<double>& data,
			const std::string& field,
			const YamlNamespace& ns);
%ignore read(std::vector<std::string> &,std::string const &);
%ignore read(std::vector<std::string> &,std::string const &,YAML::Node const &);
%rename(readStringList)
		read(std::vector<std::string>& data,
			const std::string& field,
			const YamlNamespace& ns);
%ignore read(Eigen::Vector2d &,std::string const &);
%ignore read(Eigen::Vector2d &,std::string const &,YAML::Node const &);
%rename(readArray2d) read(Eigen::Vector2d& data,
				  const std::string& field,
				  const YamlNamespace& ns);
%ignore read(Eigen::Vector3d &,std::string const &);
%ignore read(Eigen::Vector3d &,std::string const &,YAML::Node const &);
%rename(readArray3d) read(Eigen::Vector3d& data,
				  const std::string& field,
				  const YamlNamespace& ns);
%ignore read(Eigen::Quaterniond &,std::string const &);
%ignore read(Eigen::Quaterniond &,std::string const &,YAML::Node const &);
%ignore read(Pose &,std::string const &);
%ignore read(Pose &,std::string const &,YAML::Node const &);
%ignore read(Pose &,std::string const &,YamlNamespace const &);
%ignore read(Pose3d &,std::string const &);
%ignore read(Pose3d &,std::string const &,YAML::Node const &);
%ignore read(Pose3d &,std::string const &,YamlNamespace const &);
%ignore read(Action3d &,std::string const &);
%ignore read(Action3d &,std::string const &,YAML::Node const &);
%ignore read(Action3d &,std::string const &,YamlNamespace const &);
%ignore read(SearchArea &,std::string const &);
%ignore read(SearchArea &,std::string const &,YAML::Node const &);
%ignore read(SearchArea &,std::string const &,YamlNamespace const &);
%ignore getNode(YAML::Node &,YamlNamespace const &);
%ignore operator<<(YAML::Emitter&,Eigen::Vector2d const &);
%ignore operator<<(YAML::Emitter&,Eigen::Vector3d const &);

%include <dwl/utils/YamlWrapper.h>