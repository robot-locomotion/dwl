#ifndef DWL__EIGEN__H
#define DWL__EIGEN__H

#include <Eigen/Dense>
#include <map>


namespace Eigen
{

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
typedef std::map<std::string,Eigen::Vector2d> Vector2dMap;
typedef std::map<std::string,Eigen::Vector3d> Vector3dMap;
typedef std::map<std::string,Eigen::Vector6d> Vector6dMap;
typedef std::map<std::string,Eigen::Vector7d> Vector7dMap;
typedef std::map<std::string,Eigen::VectorXd> VectorXdMap;
typedef std::map<std::string,Eigen::Matrix6x> Matrix6xMap;

}


#endif
