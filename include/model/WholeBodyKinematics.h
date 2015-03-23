#ifndef DWL_WholeBodyKinematics_H
#define DWL_WholeBodyKinematics_H

#include <Eigen/Dense>
#include <map>
#include <utils/Orientation.h>
#include <utils/utils.h>
#include <utils/Math.h>
#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>


namespace dwl
{

namespace model
{

using namespace iit;

enum Component {Linear, Angular, Full};

/**
 * @brief WholeBodyKinematics class implements all the kinematics methods for a floating-base robot
 */
class WholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		WholeBodyKinematics();

		/** @brief Destructor function */
		virtual ~WholeBodyKinematics();

		/**
		 * @brief This abstract method initializes the whole-body kinematics information such as:
		 * end-effector ids and number of joints
		 */
		virtual void init() = 0;

		void computeEffectorFK(Eigen::VectorXd& position, enum Component component = Full);
		void computeEffectorFK(Eigen::VectorXd& position, EndEffectorSelector effector_set,
							   enum Component component = Full);
		Eigen::Matrix3d getBaseRotationMatrix();
		Eigen::Matrix4d getHomogeneousTransform(std::string effector_name);

		/**
		 * @brief This abstract method updates the state of the robot, which as a convention is
		 * [xb^T; q^T]^T where xb is the position and orientation of the robot base and q is joint
		 * configuration of the rigid body
		 * @param Eigen::VectorXd Current state vector
		 * @param Eigen::VectorXd Current time derivative state vector
		 */
		virtual void updateState(const iit::rbd::Vector6D& base_pos, const Eigen::VectorXd& joint_pos) = 0;

		/**
		 * @brief This method computes the whole-body jacobian for all end-effector of the robot.
		 * The whole-body jacobian represents a stack of floating-base effector jacobians in which
		 * there are base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);

		/**
		 * @brief This method computes the whole-body jacobian given a predefined set of end-effectors
		 * of the robot. The whole-body jacobian represents a stack of floating-base effector jacobians
		 * in which there are base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian, EndEffectorSelector effector_set,
											  enum Component component = Full);

		/**
		 * @brief This method computes the base jacobian contribution for all the end-effectors of the
		 * robot.
		 * @param Eigen::MatrixXd& Base jacobian
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeBaseJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);

		/**
		 * @brief This method computes the base jacobian contribution give a predefined set of end-effectors
		 * of the robot.
		 * @param Eigen::MatrixXd& Base jacobian
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeBaseJacobian(Eigen::MatrixXd& jacobian, EndEffectorSelector effector_set,
										 enum Component component = Full);

		/**
		 * @brief This method computes the effector jacobian contribution for all the end-effectors of the
		 * robot.
		 * @param Eigen::MatrixXd& Effector jacobian
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeEffectorJacobian(Eigen::MatrixXd& jacobian, enum Component component = Full);

		/**
		 * @brief This method computes the effector jacobian contribution give a predefined set of end-effectors
		 * of the robot.
		 * @param Eigen::MatrixXd& Effector jacobian
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeEffectorJacobian(Eigen::MatrixXd& jacobian, EndEffectorSelector effector_set,
											 enum Component component = Full);

		typedef std::map<std::string, Eigen::Matrix<double, 6, Eigen::Dynamic> > EndEffectorJacobian;
		typedef std::map<std::string, Eigen::Matrix<double, 4, 4> > EndEffectorHomTransform;


	protected:
		/** @brief End-effector ids */
		EndEffectorID effector_id_;

		/** @brief Jacobian matrix for the current state of the robot */
		EndEffectorJacobian jacobians_;

		/** @brief End-effector homogeneous transforms for the current state of the robot */
		EndEffectorHomTransform homogeneous_tf_;

		/** @brief Number of joints of the robot */
		int num_joints_;

		/** @brief Floating-base rotation matrix for the current state of the robot */
		Eigen::Matrix3d floating_base_rot_;
};

} //@namespace model
} //@namespace dwl

#endif
