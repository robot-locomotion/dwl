#ifndef DWL__MODEL__WHOLE_BODY_DYNAMICS__H
#define DWL__MODEL__WHOLE_BODY_DYNAMICS__H

#include <dwl/model/WholeBodyKinematics.h>
#include <dwl/model/FloatingBaseSystem.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <dwl/utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @brief WholeBodyDynamics class
 * This class various helpful methods for computing whole-body dynamics
 * quantities such as: ID and FD, constrained ID, contact forces, gravito
 * wrench, CoP, ZMP, ICP, etc. Before using this class, you need to provide
 * the floating-base system and whole-body kinematics description by passing
 * these objectsin the reset function.
 * computation.
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 */
class WholeBodyDynamics
{
	public:
		/** @brief Constructor function */
		WholeBodyDynamics();

		/** @brief Destructor function */
		~WholeBodyDynamics();

		/**
		 * @brief Set the floating-base model and the whole-body kinematics
		 * of the rigid-body system
		 * @param[in] fbs Floating-base model
		 * @param[in] wkin Whole-body kinematics
		 */
		void reset(FloatingBaseSystem& fbs,
				   WholeBodyKinematics& wkin);

		/**
		 * @brief Computes the inverse dynamics given a set of external forces
		 *
		 * The external forces can be applied in a determined body or frame.
		 * All these forces are converted into the spatial forces applied to
		 * each body. Note that the forces have to be defined in the inertial
		 * frame.
		 * @param[out] base_wrench Base wrench
		 * @param[out] joint_forces Joint forces
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration w.r.t. a gravity field [angular, linear]
		 * @param[in] joint_acc Joint acceleration
		 * @param[in] ext_force External force applied to a certain robot frame
		 */
		void computeInverseDynamics(dwl::Force& base_wrench,
									Eigen::VectorXd& joint_forces,
									dwl::SE3& base_pos,
									const Eigen::VectorXd& joint_pos,
									const dwl::Motion& base_vel,
									const Eigen::VectorXd& joint_vel,
									const dwl::Motion& base_acc,
									const Eigen::VectorXd& joint_acc,
									const dwl::ForceMap& ext_force = dwl::ForceMap());

		/**
		 * @brief Computes the inverse dynamics given a set of active contacts
		 *
		 * The active contacts define a set of holonomic constraints which are
		 * used to compute the contact forces and consistent joint
		 * accelerations. Then, these contact forces and joint accelerations
		 * are used in an ID pass for computing the joint torques.
		 * @param[out] joint_forces Joint forces
		 * @param[out] joint_acc Joint acceleration
		 * @param[out] contact_forces Contact forces
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration w.r.t. a gravity field [angular, linear]
		 * @param[in] contacts External force applied to a certain robot frame
		 */
		void computeConstrainedInverseDynamics(Eigen::VectorXd& joint_forces,
											   Eigen::VectorXd& joint_acc,
											   dwl::ForceMap& contact_forces,
											   dwl::SE3& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const dwl::Motion& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const dwl::Motion& base_acc,
											   const ElementList& contacts);

		/**
		 * @brief Computes the contact forces that generates the desired base
		 * acceleration and predefined active contacts.
		 *
		 * The desired base acceleration is used for computed a desired base
		 * wrench. Consistent joint accelerations are computed given desired
		 * active contacts (i.e. holonomic constraints).
		 * @param[out] contact_forces Contact forces applied to the defined set
		 * of contacts
		 * @param[out] joint_acc Consisten joint acceleration
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration w.r.t. a gravity field [angular, linear]
		 * @param[in] contacts Bodies that are constrained to be in contact
		 */
		void computeContactForces(dwl::ForceMap& contact_forces,
								  Eigen::VectorXd& joint_acc,
								  dwl::SE3& base_pos,
								  const Eigen::VectorXd& joint_pos,
								  const dwl::Motion& base_vel,
								  const Eigen::VectorXd& joint_vel,
								  const dwl::Motion& base_acc,
								  const ElementList& contacts);

		/**
		 * @brief Computes the joint-space inertia matrix from CRBA
		 *
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @return Joint-space inertia matrix
		 */
		const Eigen::MatrixXd&
		computeJointSpaceInertiaMatrix(dwl::SE3& base_pos,
									   const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the centroidal inertia matrix from CCRBA
		 * $hg = Ig v_{\text{mean}}$ map a mean velocity to the current
		 * centroidal momentum quantity.
		 *
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [linear, angular]
		 * @param[in] joint_vel Joint velocity
		 * @return Centroidal inertia matrix
		 */
		const Eigen::Matrix6d&
		computeCentroidalInertiaMatrix(dwl::SE3& base_pos,
									   const Eigen::VectorXd& joint_pos,
									   const dwl::Motion& base_vel,
									   const Eigen::VectorXd& joint_vel);

		/**
		 * @brief Computes the centroidal momentum matrix from CCRBA
		 * Note that $hg = A_g \dot{q}$ maps the joint velocity set to the
		 * centroidal momentum.
		 *
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [linear, angular]
		 * @param[in] joint_vel Joint velocity
		 * @return Centroidal momentum matrix
		 */
		const Eigen::Matrix6x&
		computeCentroidalMomentumMatrix(dwl::SE3& base_pos,
										const Eigen::VectorXd& joint_pos,
										const dwl::Motion& base_vel,
										const Eigen::VectorXd& joint_vel);

		/**
		 * @brief Computes the gravitational wrench in the CoM position
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @return Gravitational wrench
		 */
		const dwl::Force&
		computeGravitoWrench(dwl::SE3& base_pos,
							 const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Gets the gravitational wrench in the CoM position
		 * @param[in] com_pos CoM position expressed in the world frame
		 * @return Gravitational wrench
		 */
		const dwl::Force&
		getGravitoWrench(const Eigen::Vector3d& com_pos);

		/**
		 * @brief Computes the contact forces by comparing the estimated joint
		 * forces with the measured of the joint forces in a selected set of
		 * end-effectors
		 * @param[out] contact_forces Contact forces applied to the defined
		 * set of contacts
		 * @param[in] base_pos Base position
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration w.r.t a gravity field [angular, linear]
		 * @param[in] joint_acc Joint acceleration
		 * @param[in] joint_forces Joint forces
		 * @param[in] contacts Selected set of end-effectors (bodies)
		 */
		void estimateContactForces(dwl::ForceMap& contact_forces,
								   dwl::SE3& base_pos,
								   const Eigen::VectorXd& joint_pos,
								   const dwl::Motion& base_vel,
								   const Eigen::VectorXd& joint_vel,
								   const dwl::Motion& base_acc,
								   const Eigen::VectorXd& joint_acc,
								   const Eigen::VectorXd& joint_forces,
								   const ElementList& contacts);

		/**
		 * @brief Computes the equivalent contact forces from a center of
		 * pressure position
		 * @param[out] grfs Ground reaction forces
		 * @param[in] cop_pos Center of pressure position
		 * @param[in] contact_pos Contact positions
		 * @param[in] ground Selected set of active ground contact
		 */
		void estimateGroundReactionForces(dwl::ForceMap& grfs,
										  const Eigen::Vector3d& cop_pos,
										  const dwl::SE3Map& contact_pos,
										  const ElementList& ground_contacts);

		/**
		 * @brief Estimates active contacts by comparing the estimated joint
		 * forces with the measured joint forces in a selected set of end-effectors
		 * @param[out] active_contacts List of estimated active contacts
		 * @param[out] contact_forces Estimated contact forces
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration w.r.t. a gravity field  [angular, linear]
		 * @param[in] joint_acc Joint acceleration
		 * @param[in] joint_forces Joint forces
		 * @param[in] contact Selected set of end-effectors (bodies)
		 * @param[in] force_threshold Force threshold
		 */
		void estimateActiveContactsAndForces(ElementList& active_contacts,
											 dwl::ForceMap& contact_forces,
											 dwl::SE3& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 const dwl::Motion& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 const dwl::Motion& base_acc,
											 const Eigen::VectorXd& joint_acc,
											 const Eigen::VectorXd& joint_forces,
											 const ElementList& contacts,
											 double force_threshold);

		/**
		 * @brief Estimates active contacts by comparing the estimated joint
		 * forces with the measured joint forces in a selected set of end-effectors
		 * @param[out] active_contacts List of estimated active contacts
		 * @param[in] base_pos Base SE3 configuration
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration w.r.t. a gravity field  [angular, linear]
		 * @param[in] joint_acc Joint acceleration
		 * @param[in] joint_forces Joint forces
		 * @param[in] contact Selected set of end-effectors (bodies)
		 * @param[in] force_threshold Force threshold
		 */
		void estimateActiveContacts(ElementList& active_contacts,
									dwl::SE3& base_pos,
									const Eigen::VectorXd& joint_pos,
									const dwl::Motion& base_vel,
									const Eigen::VectorXd& joint_vel,
									const dwl::Motion& base_acc,
									const Eigen::VectorXd& joint_acc,
									const Eigen::VectorXd& joint_forces,
									const ElementList& contacts,
									double force_threshold);

		/**
		 * @brief Detects the active contacts
		 * @param[out] active_contacts Detected active contacts
		 * @param[in] contact_for Contact forces
		 * @param[in] double Force threshold
		 */
		void getActiveContacts(ElementList& active_contacs,
							   const dwl::ForceMap& contact_for,
							   double force_threshold);

		/**
		 * @brief Converts the applied external forces in a given frame to
		 * body spatial force.
		 * @param[out] body_forces Set of body forces
		 * @param[in] frame_forces External forces applied to a set of frames
		 */
		void convertAppliedExternalForces(se3::container::aligned_vector<se3::Force>& body_forces,
										  const dwl::ForceMap& frame_forces);

		/**
		 * @brief Computes the center of pressure position given the ground
		 * reactive forces and positions
		 * @param[out] cop_pos Center of pressure position
		 * @param[in] contact_for Contact forces
		 * @param[in] contact_pos Contact position
		 */
		void computeCenterOfPressure(Eigen::Vector3d& cop_pos,
									 const dwl::ForceMap& contact_for,
									 const dwl::SE3Map& contact_pos);

		/**
		 * @brief Computes the Zero Moment Point (ZMP) assuming an inverted
		 * pendulum model
		 * @param[out] zmp_pos ZMP position
		 * @param[in] com_pos Center of mass position
		 * @param[in] com_acc Center of mass acceleration
		 * @param[in] height Pendulum height
		 */
		void computeZeroMomentPoint(Eigen::Vector3d& zmp_pos,
									const Eigen::Vector3d& com_pos,
									const Eigen::Vector3d& com_acc,
									const double& height);

		/**
		 * @brief Computes the Instantaneous Capture Point (ICP) assuming an
		 * inverted pendulum model
		 * @param[out] icp_pos ICP position
		 * @param[in] com_pos CoM position
		 * @param[in] com_vel CoM velocity
		 * @param[in] height Pendulum height
		 */
		void computeInstantaneousCapturePoint(Eigen::Vector3d& icp_pos,
				                              const Eigen::Vector3d& com_pos,
											  const Eigen::Vector3d& com_vel,
											  const double& height);

		/**
		 * @brief Computes the centroidal moment pivot position given the
		 * contact forces (i.e. GRFs)
		 * @param[out] cmp_pos Centroidal moment pivot position
		 * @param[in] com_pos CoM position
		 * @param[in] height Pendulum height
		 * @param[in] contact_for Contact forces
		 */
		void computeCentroidalMomentPivot(Eigen::Vector3d& cmp_pos,
				                          const Eigen::Vector3d& com_pos,
				                          const double& height,
				                          const dwl::ForceMap& contact_for);

		/**
		 * @brief Computes the CoM torque given the CoP and CMP positions
		 * @param[out] torque CoM torque
		 * @param[in] cop_pos CoP position
		 * @param[in] cmp_pos CMP position
		 * @param[in] contact_for Contact forces
		 */
		void computeCoMTorque(Eigen::Vector3d& torque,
							  const Eigen::Vector3d& cop_pos,
							  const Eigen::Vector3d& cmp_pos,
							  const dwl::ForceMap& contact_for);

		/** @brief Gets the floating-base system information */
		std::shared_ptr<FloatingBaseSystem> getFloatingBaseSystem();

		/** @brief Gets the whole-body kinematics */
		std::shared_ptr<WholeBodyKinematics> getWholeBodyKinematics();


	private:
		/** @brief Kinematic model */
		std::shared_ptr<WholeBodyKinematics> wkin_;

		/** @brief A floating-base system information */
		std::shared_ptr<FloatingBaseSystem> fbs_;

		/** @brief Set of body spatial forces */
		se3::container::aligned_vector<se3::Force> body_forces_;

		/** @brief Gravitational wrench */
		dwl::Force grav_wrench_;

		/** @brief The centroidal inertia matrix */
		Eigen::Matrix6d com_inertia_mat_;
};

} //@namespace model
} //@namespace dwl

#endif
