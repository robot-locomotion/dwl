%module dwl
%{
#include <Python.h>
#include <dwl/WholeBodyState.h>
#include <dwl/utils/URDF.h>
#include <dwl/model/FloatingBaseSystem.h>
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
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

%template(vector_matrix3d) std::vector<Eigen::Matrix3d>;
%template(vector_matrix4d) std::vector<Eigen::Matrix4d>;
%template(vector_matrixXd) std::vector<Eigen::MatrixXd>;
%template(vector_vector2d) std::vector<Eigen::Vector2d>;
%template(vector_vector3d) std::vector<Eigen::Vector3d>;
%template(vector_vectorXd) std::vector<Eigen::VectorXd>;
%template(string_jointLimits) std::map<std::string,urdf::JointLimits>; 


%include <dwl/utils/RigidBodyDynamics.h>
%include <dwl/WholeBodyState.h>
%include <dwl/utils/URDF.h>
%include <dwl/model/FloatingBaseSystem.h>
