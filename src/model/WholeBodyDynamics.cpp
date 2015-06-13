#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

WholeBodyDynamics::WholeBodyDynamics()
{

}


WholeBodyDynamics::~WholeBodyDynamics()
{

}


void WholeBodyDynamics::modelFromURDF(std::string model_file, bool info)
{
	RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), &robot_model_, false);

	if (info) {
		std::cout << "Degree of freedom overview:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot_model_);
		std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(robot_model_);

		std::cout << "Model Hierarchy:" << std::endl;
		std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(robot_model_);
	}
}


void WholeBodyDynamics::computeWholeBodyInverseDynamics(rbd::Vector6d& base_wrench,
															   Eigen::VectorXd& joint_forces,
															   const rbd::Vector6d& base_pos,
															   const Eigen::VectorXd& joint_pos,
															   const rbd::Vector6d& base_vel,
															   const Eigen::VectorXd& joint_vel,
															   const rbd::Vector6d& base_acc,
															   const Eigen::VectorXd& joint_acc,
															   const rbd::EndEffectorForce& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::InverseDynamics(robot_model_, q, q_dot, q_ddot, tau, &fext);

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau);
}



void WholeBodyDynamics::computeWholeBodyInverseDynamics(rbd::Vector6d& base_acc,
															   Eigen::VectorXd& joint_forces,
															   const rbd::Vector6d& base_pos,
															   const Eigen::VectorXd& joint_pos,
															   const rbd::Vector6d& base_vel,
															   const Eigen::VectorXd& joint_vel,
															   const Eigen::VectorXd& joint_acc,
															   const rbd::EndEffectorForce& ext_force)
{
	// Converting base and joint states to generalized joint states
	Eigen::VectorXd q = rbd::toGeneralizedJointState(robot_model_, base_pos, joint_pos);
	Eigen::VectorXd q_dot = rbd::toGeneralizedJointState(robot_model_, base_vel, joint_vel);
	Eigen::VectorXd q_ddot = rbd::toGeneralizedJointState(robot_model_, base_acc, joint_acc);
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_model_.dof_count);

	// Computing the applied external spatial forces for every body
	std::vector<SpatialVector_t> fext;
	convertAppliedExternalForces(fext, ext_force, q);

	// Computing the inverse dynamics with Recursive Newton-Euler Algorithm (RNEA)
	RigidBodyDynamics::Math::SpatialVector base_ddot = RigidBodyDynamics::Math::SpatialVector(base_acc);
	FloatingBaseInverseDynamics(robot_model_, q, q_dot, q_ddot, base_ddot, tau, &fext);

	// Converting the base acceleration
	base_acc = base_ddot;

	// Converting the generalized joint forces to base wrench and joint forces
	rbd::Vector6d base_wrench;
	rbd::fromGeneralizedJointState(robot_model_, base_wrench, joint_forces, tau);
}


void WholeBodyDynamics::computeConstrainedWholeBodyInverseDynamics(Eigen::VectorXd& joint_forces,
																			const rbd::Vector6d& base_pos,
																			const Eigen::VectorXd& joint_pos,
																			const rbd::Vector6d& base_vel,
																			const Eigen::VectorXd& joint_vel,
																			const rbd::Vector6d& base_acc,
																			const Eigen::VectorXd& joint_acc,
																			const rbd::EndEffectorSelector& contacts)
{

}


void WholeBodyDynamics::FloatingBaseInverseDynamics(RigidBodyDynamics::Model& model,
														   const RigidBodyDynamics::Math::VectorNd &Q,
														   const RigidBodyDynamics::Math::VectorNd &QDot,
														   const RigidBodyDynamics::Math::VectorNd &QDDot,
														   RigidBodyDynamics::Math::SpatialVector &base_acc,
														   RigidBodyDynamics::Math::VectorNd &Tau,
														   std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	LOG << "-------- " << __func__ << " --------" << std::endl;

	// First pass
	for (unsigned int i = 1; i < 7; i++) {
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);
		model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
	}

	for (unsigned int i = 7; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];
		jcalc (model, i, Q, QDot);

		model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];

		model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);

		if (model.mJoints[i].mDoFCount == 3) {
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i] +
					model.multdof3_S[i] * Vector3d (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
		} else {
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i] + model.S[i] * QDDot[q_index];
		}

		model.Ic[i] = model.I[i];

		if (!model.mBodies[i].mIsVirtual) {
			model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
		} else {
			model.f[i].setZero();
		}

		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	// Second pass
	model.Ic[6] = model.I[6];
	model.f[6] = model.I[6] * model.a[6] + crossf(model.v[6],model.I[6] * model.v[6]);
	if (f_ext != NULL && (*f_ext)[6] != SpatialVectorZero)
		model.f[6] -= (*f_ext)[6];

	for (unsigned int i = model.mBodies.size() - 1; i > 6; i--) {
		unsigned int lambda = model.lambda[i];

		model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].apply(model.Ic[i]);
		model.f[lambda] = model.f[lambda] + model.X_lambda[i].applyTranspose(model.f[i]);
	}

	// Third pass
	model.a[6] = - model.Ic[6].toMatrix().inverse() * model.f[6];

	for (unsigned int i = 7; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		model.a[i] = model.X_lambda[i].apply(model.a[lambda]);

		if (model.mJoints[i].mDoFCount == 3) {
			Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() *
					(model.Ic[i] * model.a[i] + model.f[i]);
		} else {
			Tau[model.mJoints[i].q_index] = model.S[i].dot(model.Ic[i] * model.a[i] + model.f[i]);
		}
	}

	base_acc = model.a[6];
}


void WholeBodyDynamics::convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& fext,
													 const rbd::EndEffectorForce& ext_force,
													 const Eigen::VectorXd& q)
{
	// Computing the applied external spatial forces for every body
	if (ext_force.size() > 0) {
		//fext = new std::vector<SpatialVector_t>();
		fext.resize(robot_model_.mBodies.size());
		// Searching over the movable bodies
		for (unsigned int body_id = 0; body_id < robot_model_.mBodies.size(); body_id++) {
			std::string body_name = robot_model_.GetBodyName(body_id);

			if (ext_force.count(body_name) > 0) {
				// Converting the applied force to spatial force vector in base coordinates
				rbd::Vector6d force = ext_force.at(body_name);
				Eigen::Vector3d force_point =
						CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
				rbd::Vector6d spatial_force = rbd::convertForceToSpatialForce(force, force_point);
				fext.at(body_id) = spatial_force;
			} else
				fext.at(body_id) = rbd::Vector6d::Zero();
		}

		// Searching over the fixed bodies
		for (unsigned int it = 0; it < robot_model_.mFixedBodies.size(); it++) {
			unsigned int body_id = it + robot_model_.fixed_body_discriminator;
			std::string body_name = robot_model_.GetBodyName(body_id);
			if (ext_force.count(body_name) > 0) {
				// Converting the applied force to spatial force vector in base coordinates
				rbd::Vector6d force = ext_force.at(body_name);
				Eigen::Vector3d force_point =
						CalcBodyToBaseCoordinates(robot_model_, q, body_id, Eigen::Vector3d::Zero(), true);
				rbd::Vector6d spatial_force = rbd::convertForceToSpatialForce(force, force_point);

				unsigned parent_id = robot_model_.mFixedBodies[it].mMovableParent;
				fext.at(parent_id) += spatial_force;
			}
		}
	}
}

} //@namespace model
} //@namespace dwl
