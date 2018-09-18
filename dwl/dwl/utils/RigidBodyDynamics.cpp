#include <dwl/utils/RigidBodyDynamics.h>


namespace dwl
{

namespace rbd
{

std::string coord3dToName(enum Coords3d coord)
{
	const char* Coord3dNames[3] = {"X", "Y", "Z"};

	std::string coord_name(Coord3dNames[coord]);
	return coord_name;
}


std::string coord6dToName(enum Coords6d coord)
{
	const char* Coord6dNames[6] = {"LX_V", "LY_V", "LZ_V",
								   "AX_V", "AY_V", "AZ_V"};

	std::string coord_name(Coord6dNames[coord]);
	return coord_name;
}


std::string coord7dToName(enum Coords7d coord)
{
	const char* Coord7dNames[7] = {"LX_Q", "LY_Q", "LZ_Q",
								   "AX_Q", "AY_Q", "AZ_Q", "AW_Q"};

	std::string coord_name(Coord7dNames[coord]);
	return coord_name;
}


Eigen::Vector3d angularPart(Eigen::Vector6d& vector)
{
	return vector.segment<3>(AX_V);
}


Eigen::Vector4d angularPart(Eigen::Vector7d& vector)
{
	return vector.segment<4>(AX_Q);
}


Eigen::Vector3d linearPart(Eigen::Vector6d& vector)
{
	return vector.segment<3>(LX_V);
}


Eigen::Vector3d linearPart(Eigen::Vector7d& vector)
{
	return vector.segment<3>(LX_Q);
}


Eigen::Vector3d translationVector(Eigen::Matrix4d& hom_transform)
{
    return hom_transform.block<3,1>(0,3);
}


Eigen::Matrix3d rotationMatrix(Eigen::MatrixBase<Eigen::Matrix4d>& hom_transform)
{
    return hom_transform.block<3,3>(0,0);
}

} //@namespace rbd
} //@namespace dwl
