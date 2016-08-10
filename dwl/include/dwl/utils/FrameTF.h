#ifndef DWL__MATH__FRAME_TF__H
#define DWL__MATH__FRAME_TF__H

#include <dwl/utils/Orientation.h>


namespace dwl
{

namespace math
{

/**
 * @class the FrameTF class
 * This class provides an user friendly interface for transforming different
 * frame: world, base and horizontal frames.
 */
class FrameTF
{
	public:
		/** @brief Constructor function */
		FrameTF();

		/** @brief Destructor function */
		~FrameTF();

		/** @brief Transforms the defined vector in the world into the base
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d fromWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											 const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d fromWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											 const Eigen::Quaterniond& q) const;

		/** @brief Transforms the defined vector in the world into the horizontal
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d fromWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
												   const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d fromWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
												   const Eigen::Quaterniond& q) const;

		/** @brief Transforms the defined vector in the base into the world
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d fromBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											 const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d fromBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											 const Eigen::Quaterniond& q) const;

		/** @brief Transforms the defined vector in the base into the horizontal
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d fromBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												  const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d fromBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												  const Eigen::Quaterniond& q) const;

		/** @brief Transforms the defined vector in the horizontal into the world
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d fromHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d fromHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Quaterniond& q) const;

		/** @brief Transforms the defined vector in the horizontal into the base
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d fromHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												  const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d fromHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												  const Eigen::Quaterniond& q) const;


		/** @brief Maps the defined vector in the world frame into the base
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d mapWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d mapWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											const Eigen::Quaterniond& q) const;

		/** @brief Maps the defined vector in the base frame into the world
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d mapBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d mapBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											const Eigen::Quaterniond& q) const;

		/** @brief Maps the defined vector in the world frame into the horizontal
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d mapWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
												  const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d mapWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
												  const Eigen::Quaterniond& q) const;

		/** @brief Maps the defined vector in the horizontal frame into the world
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d mapHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
												  const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d mapHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
												  const Eigen::Quaterniond& q) const;

		/** @brief Maps the defined vector in the base frame into the horizontal
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d mapBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												 const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d mapBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												 const Eigen::Quaterniond& q) const;

		/** @brief Maps the defined vector in the horizontal frame into the base
		 * frame given the orientation of the body: RPY angles or quaternion */
		Eigen::Vector3d mapHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												 const Eigen::Vector3d& rpy) const;
		Eigen::Vector3d mapHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												 const Eigen::Quaterniond& rpy) const;


	private:
		/** @brief Computes the rotation matrix from world to horizontal frame */
		Eigen::Matrix3d inline getRotHorizontalToWorld(const Eigen::Vector3d& rpy) const {
			// Note that the roll and pitch components are zero w.r.t. the
			// world frame
			Eigen::Vector3d rpy_H(0., 0., rpy(2));
			return math::getRotationMatrix(rpy_H);
		}

		/** @brief Computes the rotation matrix from world to horizontal frame */
		Eigen::Matrix3d inline getRotBaseToHF(const Eigen::Vector3d& rpy) const {
			// Note that the yaw component is zero w.r.t. the base frame
			Eigen::Matrix3d R;

			R <<  cos(rpy(1)),  sin(rpy(0))*sin(rpy(1)),  cos(rpy(0))*sin(rpy(1)),
					       0.,              cos(rpy(0)),             -sin(rpy(0)),
				 -sin(rpy(1)),  sin(rpy(0))*cos(rpy(1)),  cos(rpy(0))*cos(rpy(1));

			return R;
		}
};

} //@namespace math
} //@namespace dwl

#endif
