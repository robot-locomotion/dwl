%module dwl
%{
#include <vector>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <dwl/utils/Math.h>
//#include <dwl/utils/FrameTF.h>
#include <dwl/utils/RigidBodyDynamics.h>
%}

/// Data structure in the target language holding data
#ifdef SWIGPYTHON
#define GUESTOBJECT PyObject
#elif defined(SWIGMATLAB)
#define GUESTOBJECT mxArray
#else
#define GUESTOBJECT void
#endif

%include "std_vector.i"
%include <eigen.i>
%rename(myVector6d) dwl::rbd::Vector6d;
%rename(myPart3d) dwl::rbd::Part3d;
typedef Eigen::Block<Vector6d,3,1> Part3d;

//%ignore angularPart(Vector6d& vector);
//myPart3d angularPart(myVector6d& vector);


//%include <rbdl/rbdl.h>
%include <dwl/utils/Math.h>

%include <dwl/utils/RigidBodyDynamics.h>
//%include "dwl/WholeBodyState.h"



%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorVectorXd) std::vector<Eigen::VectorXd>;

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
// Even though Eigen::MatrixXd is just a typedef for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>, our templatedInverse function doesn't
// compile correctly unless we also declare typemaps for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>. Not totally sure why that is.
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

