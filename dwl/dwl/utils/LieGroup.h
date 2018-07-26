#ifndef DWL__LIE_GROUP__H
#define DWL__LIE_GROUP__H

#include <dwl/utils/EigenExtra.h>
#include <dwl/utils/Orientation.h>
#include <pinocchio/multibody/liegroup/liegroup.hpp>


#define SE3_ORIGIN dwl::SE3()
#define ZERO_MOTION dwl::Motion()
#define ZERO_FORCE dwl::Motion()


namespace dwl
{

/**
 * @brief SE3 class
 * This class describes Special Euclidean group for SE(3). It allows us to
 * get and to set its translation and rotation. As in SE(3), the rotation is
 * defined using the rotation matrix, however it possible to set (or to get)
 * its representation in RPY angles and quaternion. Note that the quaternion
 * representation is used whenever we want to obtain the vector. The operations
 * of this group are described in se3::SE3 class.
 * @copyright BSD 3-Clause License
 */
struct SE3
{
	/** @brief Constructs the SE(3) in its origin */
	SE3() : data(1) {}

	/**
	 * @brief Constructs the SE(3) given (t,R)
	 *
	 * @param[in] t Translation
	 * @param[in] R Rotation matrix
	 */
	SE3(const Eigen::Vector3d& t,
		const Eigen::Matrix3d& R) : data(R, t) {}

	/**
	 * @brief Constructs the SE(3) using the quaternion representation
	 *
	 * @param[in] t Translation
	 * @param[in] q Quaternion
	 */
	SE3(const Eigen::Vector3d& t,
		const Eigen::Vector4d& q) {
		setTranslation(t);
		setQuaternion(q);
	}

	/**
	 * @brief Constructs the SE(3) using the RPY angles representation
	 *
	 * @param[in] t Translation
	 * @param[in] rpy RPY angles
	 */
	SE3(const Eigen::Vector3d& t,
		const Eigen::Vector3d& rpy) {
		setTranslation(t);
		setRPY(rpy);
	}

	/**
	 * @brief Gets the 7D vector that represents the SE(3)
	 * This vector uses the quaternion representation since it doesn't suffer
	 * from singularity.
	 *
	 * @return [Translation, Quaternion]
	 */
	const Eigen::Vector7d& toVector() {
		vec << getTranslation(), getQuaternion();
		return vec;
	}

	/**
	 * @brief Gets the translation
	 *
	 * @return Translation
	 */
	const Eigen::Vector3d& getTranslation() const {
		return data.translation();
	}

	/**
	 * @brief Gets the rotation matrix
	 *
	 * @return Rotation matrix
	 */
	const Eigen::Matrix3d& getRotation() const {
		return data.rotation();
	}

	/**
	 * @brief Gets the quaternion
	 *
	 * @return Quaternion [x,y,z,w]
	 */
	const Eigen::Vector4d& getQuaternion() {
		q1 = math::getQuaternion(getRotation());
		q2 << q1.x(), q1.y(), q1.z(), q1.w();
		return q2;
	}

	/**
	 * @brief Gets the RPY angles
	 *
	 * @return RPY angles
	 */
	const Eigen::Vector3d& getRPY() {
		rpy = math::getRPY(getRotation());
		return rpy;
	}

	/**
	 * @brief Sets the translation
	 *
	 * @param[in] t Translation
	 */
	void setTranslation(const Eigen::Vector3d& t) {
		data.translation(t);
	}

	/**
	 * @brief Sets the rotation matrix
	 *
	 * @param[in] R Rotation matrix
	 */
	void setRotation(const Eigen::Matrix3d& R) {
		data.rotation(R);
	}

	/**
	 * @brief Sets quaternion
	 *
	 * @param[in] q Quaternion [x,y,z,w]
	 */
	void setQuaternion(const Eigen::Vector4d& q) {
		 // Note that DWL convention is XYZW, instead Eigen WXYZ
		q1 = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
		data.rotation(math::getRotationMatrix(q1));
	}

	/**
	 * @brief Sets RPY angles
	 *
	 * @param[in] rpy RPY angles
	 */
	void setRPY(const Eigen::Vector3d& rpy) {
		data.rotation(math::getRotationMatrix(rpy));
	}

	se3::SE3 data;

private:
	Eigen::Vector7d vec;
	Eigen::Quaterniond q1;
	Eigen::Vector4d q2;
	Eigen::Vector3d rpy;
};


/**
 * @brief Motion class
 * This class describes spatial velocities, elements of SE(3) = M^6. It allows
 * us to get and to set its linear and angular components. The operations of
 * this group are described in se3::Motion class.
 * @copyright BSD 3-Clause License
 */
struct Motion
{
	/** @brief Constructs the motion in its origin */
	Motion() {
		data.setZero();
	}

	/**
	 * @brief Constructs the motion give (v,w)
	 *
	 * @param[in] v Linear component
	 * @param[in] w Angular component
	 */
	Motion(const Eigen::Vector3d& v,
		   const Eigen::Vector3d& w) : data(v, w) {}

	/**
	 * @brief Gets the 6D vector that represents the motion
	 *
	 * @return [Linear, Angular]
	 */
	const Eigen::Vector6d& toVector() const {
		return data.toVector();
	}

	/**
	 * @brief Gets the linear component
	 *
	 * @return Linear component
	 */
	Eigen::Vector3d getLinear() const {
		return data.linear();
	}

	/**
	 * @brief Gets the angular component
	 *
	 * @return Angular component
	 */
	Eigen::Vector3d getAngular() const {
		return data.angular();
	}

	/**
	 * @brief Sets the linear component
	 *
	 * @param[in] v Linear component
	 */
	void setLinear(const Eigen::Vector3d& v) {
		return data.linear(v);
	}

	/**
	 * @brief Sets the angular component
	 *
	 * @param[in] w Angular component
	 */
	void setAngular(const Eigen::Vector3d& w) {
		return data.angular(w);
	}

	se3::Motion data;
};


/**
 * @brief Force class
 * This class describes spatial force, elements of SE(3) = F^6. It allows
 * us to get and to set its linear and angular components. The operations of
 * this group are described in se3::Force class.
 * @copyright BSD 3-Clause License
 */
struct Force
{
	/** @brief Constructs the motion in its origin */
	Force() {
		data.setZero();
	}

	/**
	 * @brief Constructs the motion give (v,w)
	 *
	 * @param[in] f Linear component
	 * @param[in] n Angular component
	 */
	Force(const Eigen::Vector3d& f,
		  const Eigen::Vector3d& n) : data(f, n) {}

	/**
	 * @brief Gets the 6D vector that represents the motion
	 *
	 * @return [Linear, Angular]
	 */
	const Eigen::Vector6d& toVector() const {
		return data.toVector();
	}

	/**
	 * @brief Gets the linear component
	 *
	 * @return Linear component
	 */
	Eigen::Vector3d getLinear() const {
		return data.linear();
	}

	/**
	 * @brief Gets the angular component
	 *
	 * @return Angular component
	 */
	Eigen::Vector3d getAngular() const {
		return data.angular();
	}

	/**
	 * @brief Sets the linear component
	 *
	 * @param[in] v Linear component
	 */
	void setLinear(const Eigen::Vector3d& v) {
		return data.linear(v);
	}

	/**
	 * @brief Sets the angular component
	 *
	 * @param[in] w Angular component
	 */
	void setAngular(const Eigen::Vector3d& w) {
		return data.angular(w);
	}

	se3::Force data;
};

typedef std::map<std::string,dwl::SE3> SE3Map;
typedef std::map<std::string,dwl::Motion> MotionMap;
typedef std::map<std::string,dwl::Force> ForceMap;
}

#endif
