#include <utils/RigidBodyDynamics.h>


namespace dwl
{

namespace rbd
{

Part3d angularPart(Vector6d& vector)
{
	return vector.topRows<3>();
}


Part3d linearPart(Vector6d& vector)
{
	return vector.bottomRows<3>();
}


TranslationPart translationVector(Eigen::Matrix4d& hom_transform)
{
    return hom_transform.block<3,1>(0,3);
}


RotationPart rotationMatrix(Eigen::MatrixBase<Eigen::Matrix4d>& hom_transform)
{
    return hom_transform.block<3,3>(0,0);
}


void getListOfBodies(BodyID& list_body_id,
					 const RigidBodyDynamics::Model& model)
{
	for (unsigned int it = 0; it < model.mBodies.size(); it++) {
		unsigned int body_id = it;
		std::string body_name = model.GetBodyName(body_id);

		list_body_id[body_name] = body_id;
	}

	// Adding the fixed body in the body list
	for (unsigned int it = 0; it < model.mFixedBodies.size(); it++) {
		unsigned int body_id = it + model.fixed_body_discriminator;
		std::string body_name = model.GetBodyName(body_id);

		list_body_id[body_name] = body_id;
	}
}


void printModelInfo(const RigidBodyDynamics::Model& model)
{
	std::cout << "Degree of freedom overview:" << std::endl;
	std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(model);

	std::cout << "Body origins overview:" << std::endl;
	RigidBodyDynamics::Model copy_model = model;
	std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(copy_model);

	std::cout << "Model Hierarchy:" << std::endl;
	std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(model);
}


Eigen::VectorXd toGeneralizedJointState(const Vector6d& base_state,
										const Eigen::VectorXd& joint_state,
										struct FloatingBaseSystem& system)
{
	// Getting the number of joints
	assert(joint_state.size() == system.getJointDoF());

	// Note that RBDL defines the floating base state as [linear states, angular states]
	Eigen::VectorXd q;
	if (system.getTypeOfDynamicSystem() == FloatingBase ||
			system.getTypeOfDynamicSystem() == ConstrainedFloatingBase) {
		q.resize(6 + system.getJointDoF());

		Vector6d _base_state = base_state;
		q << linearPart(_base_state), angularPart(_base_state), joint_state;
	} else if (system.getTypeOfDynamicSystem() == VirtualFloatingBase) {
		unsigned int base_dof = system.getFloatingBaseDoF();
		q.resize(base_dof + system.getJointDoF());

		Eigen::VectorXd virtual_base(base_dof);
		if (system.AX.active)
			virtual_base(system.AX.id) = base_state(AX);
		if (system.AY.active)
			virtual_base(system.AY.id) = base_state(AY);
		if (system.AZ.active)
			virtual_base(system.AZ.id) = base_state(AZ);
		if (system.LX.active)
			virtual_base(system.LX.id) = base_state(LX);
		if (system.LY.active)
			virtual_base(system.LY.id) = base_state(LY);
		if (system.LZ.active)
			virtual_base(system.LZ.id) = base_state(LZ);

		q << virtual_base, joint_state;
	} else {
		q.resize(system.getJointDoF());
		q = joint_state;
	}

	return q;
}


void fromGeneralizedJointState(Vector6d& base_state,
							   Eigen::VectorXd& joint_state,
							   const Eigen::VectorXd& generalized_state,
							   struct FloatingBaseSystem& system)
{
	// Resizing the joint state
	joint_state.resize(system.getJointDoF());

	// Note that RBDL defines the floating base state as [linear states, angular states]
	if (system.getTypeOfDynamicSystem() == FloatingBase ||
			system.getTypeOfDynamicSystem() == ConstrainedFloatingBase) {
		base_state << generalized_state.segment<3>(rbd::LX), generalized_state.segment<3>(rbd::AX);
		joint_state = generalized_state.segment(6, system.getJointDoF());
	} else if (system.getTypeOfDynamicSystem() == VirtualFloatingBase) {
		if (system.AX.active)
			base_state(AX) = generalized_state(system.AX.id);
		if (system.AY.active)
			base_state(AY) = generalized_state(system.AY.id);
		if (system.AZ.active)
			base_state(AZ) = generalized_state(system.AZ.id);
		if (system.LX.active)
			base_state(LX) = generalized_state(system.LX.id);
		if (system.LY.active)
			base_state(LY) = generalized_state(system.LY.id);
		if (system.LZ.active)
			base_state(LZ) = generalized_state(system.LZ.id);

		joint_state = generalized_state.segment(system.getFloatingBaseDoF(), system.getJointDoF());
	} else {
		base_state = Vector6d::Zero();
		joint_state = generalized_state;
	}
}


void setBranchState(Eigen::VectorXd& new_joint_state,
					const Eigen::VectorXd& branch_state,
					unsigned int body_id,
					struct FloatingBaseSystem& system)
{
	// Getting the base joint id. Note that the floating-base starts the kinematic-tree
	unsigned int base_id = 0;
	for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
		dwl::rbd::Coords6d base_coord = dwl::rbd::Coords6d(base_idx);
		dwl::rbd::FloatingBaseJoint base_joint = system.getFloatingBaseJoint(base_coord);
		if (base_joint.active) {
			if (base_joint.id > base_id)
				base_id = base_joint.id;
		}
	}

	// Setting the state values of a specific branch to the joint state
	unsigned int parent_id = body_id;
	if (system.rbd_model.IsFixedBodyId(body_id)) {
		unsigned int fixed_idx = system.rbd_model.fixed_body_discriminator;
		parent_id = system.rbd_model.mFixedBodies[body_id - fixed_idx].mMovableParent;
	}

	// Adding the branch state to the joint state. Two safety checking are done; checking that this
	// branch has at least one joint, and checking the size of the new branch state
	if (parent_id != base_id) {
		unsigned int q_index, num_dof = 0;
		do {
			q_index = system.rbd_model.mJoints[parent_id].q_index - 1;
			parent_id = system.rbd_model.lambda[parent_id];
			++num_dof;
		} while (parent_id != base_id);

		assert(branch_state.size() == num_dof);
		new_joint_state.segment(q_index, num_dof) = branch_state;
	}
}


Eigen::VectorXd getBranchState(Eigen::VectorXd& joint_state,
							   unsigned int body_id,
							   struct FloatingBaseSystem& system)
{
	unsigned int q_index, num_dof = 0;

	// Getting the base joint id. Note that the floating-base starts the kinematic-tree
	unsigned int base_id = 0;
	for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
		dwl::rbd::Coords6d base_coord = dwl::rbd::Coords6d(base_idx);
		dwl::rbd::FloatingBaseJoint base_joint = system.getFloatingBaseJoint(base_coord);
		if (base_joint.active) {
			if (base_joint.id > base_id)
				base_id = base_joint.id;
		}
	}

	// Setting the state values of a specific branch to the joint state
	unsigned int parent_id = body_id;
	if (system.rbd_model.IsFixedBodyId(body_id)) {
		unsigned int fixed_idx = system.rbd_model.fixed_body_discriminator;
		parent_id = system.rbd_model.mFixedBodies[body_id - fixed_idx].mMovableParent;
	}

	// Adding the branch state to the joint state. Two safety checking are done; checking that this
	// branch has at least one joint, and checking the size of the new branch state
	if (parent_id != base_id) {
		do {
			q_index = system.rbd_model.mJoints[parent_id].q_index - 1;
			parent_id = system.rbd_model.lambda[parent_id];
			++num_dof;
		} while (parent_id != base_id);
	}

	Eigen::VectorXd branch_state(num_dof);
	branch_state = joint_state.segment(q_index, num_dof);

	return branch_state;
}


Vector6d convertPointVelocityToSpatialVelocity(Vector6d& velocity,
											   const Eigen::Vector3d& point)
{
	rbd::Vector6d spatial_velocity;
	spatial_velocity.segment(rbd::AX,3) = angularPart(velocity);
	spatial_velocity.segment(rbd::LX,3) = linearPart(velocity) +
	math::skewSymmentricMatrixFrom3DVector(point) * angularPart(velocity);

	return spatial_velocity;
}


Vector6d convertPointForceToSpatialForce(Vector6d& force,
										 const Eigen::Vector3d& point)
{
	rbd::Vector6d spatial_force;
	spatial_force.segment(rbd::AX,3) = angularPart(force) +
			math::skewSymmentricMatrixFrom3DVector(point) * linearPart(force);
	spatial_force.segment(rbd::LX,3) = linearPart(force);

	return spatial_force;
}


void computePointJacobian(RigidBodyDynamics::Model& model,
						  const RigidBodyDynamics::Math::VectorNd &Q,
						  unsigned int body_id,
						  const RigidBodyDynamics::Math::Vector3d& point_position,
						  RigidBodyDynamics::Math::MatrixNd& G,
						  bool update_kinematics)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;
	LOG << "-------- " << __func__ << " --------" << std::endl;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom(model, &Q, NULL, NULL);
	}

	SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(),
			CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false));

	assert (G.rows() == 6 && G.cols() == model.qdot_size );

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.
	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		if (model.mJoints[j].mDoFCount == 3) {
			G.block(0,q_index,6,3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]);
		} else {
			G.block(0,q_index,6,1) = point_trans.apply(model.X_base[j].inverse().apply(model.S[j]));
		}

		j = model.lambda[j];
	}
}


rbd::Vector6d computePointVelocity(RigidBodyDynamics::Model& model,
								   const RigidBodyDynamics::Math::VectorNd& Q,
								   const RigidBodyDynamics::Math::VectorNd& QDot,
								   unsigned int body_id,
								   const RigidBodyDynamics::Math::Vector3d point_position,
								   bool update_kinematics)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	LOG << "-------- " << __func__ << " --------" << std::endl;
	assert (model.IsBodyId(body_id));
	assert (model.q_size == Q.size());
	assert (model.qdot_size == QDot.size());

	// Reset the velocity of the root body
	model.v[0].setZero();

	// update the Kinematics with zero acceleration
	if (update_kinematics) {
		UpdateKinematicsCustom(model, &Q, &QDot, NULL);
	}

	unsigned int reference_body_id = body_id;
	Vector3d reference_point = point_position;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
		Vector3d base_coords = CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false);
		reference_point = CalcBaseToBodyCoordinates(model, Q, reference_body_id, base_coords, false);
	}

	SpatialVector point_spatial_velocity =
			SpatialTransform(CalcBodyWorldOrientation(model, Q, reference_body_id, false).transpose(),
					reference_point).apply(model.v[reference_body_id]);

	return point_spatial_velocity;
}


rbd::Vector6d computePointAcceleration(RigidBodyDynamics::Model& model,
									   const RigidBodyDynamics::Math::VectorNd& Q,
									   const RigidBodyDynamics::Math::VectorNd& QDot,
									   const RigidBodyDynamics::Math::VectorNd& QDDot,
									   unsigned int body_id,
									   const Eigen::Vector3d point_position,
									   bool update_kinematics)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	LOG << "-------- " << __func__ << " --------" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0].setZero();

	if (update_kinematics)
		UpdateKinematics(model, Q, QDot, QDDot);

	LOG << std::endl;

	unsigned int reference_body_id = body_id;
	Vector3d reference_point = point_position;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
		Vector3d base_coords = CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false);
		reference_point = CalcBaseToBodyCoordinates(model, Q, reference_body_id, base_coords, false);
	}

	SpatialTransform p_X_i (CalcBodyWorldOrientation(model, Q, reference_body_id, false).transpose(), reference_point);

	SpatialVector p_v_i = p_X_i.apply(model.v[reference_body_id]);
	Vector3d a_dash = Vector3d (p_v_i[0], p_v_i[1], p_v_i[2]).cross(Vector3d (p_v_i[3], p_v_i[4], p_v_i[5]));
	SpatialVector p_a_i = p_X_i.apply(model.a[reference_body_id]);

	rbd::Vector6d point_spatial_acceleration;
	point_spatial_acceleration << p_a_i[0], p_a_i[1], p_a_i[2],
								  p_a_i[3] + a_dash[0], p_a_i[4] + a_dash[1], p_a_i[5] + a_dash[2];

	return point_spatial_acceleration;
}


void FloatingBaseInverseDynamics(RigidBodyDynamics::Model& model,
								 const RigidBodyDynamics::Math::VectorNd &Q,
								 const RigidBodyDynamics::Math::VectorNd &QDot,
								 const RigidBodyDynamics::Math::VectorNd &QDDot,
								 RigidBodyDynamics::Math::SpatialVector &base_acc,
								 RigidBodyDynamics::Math::VectorNd &Tau,
								 std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext)
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	// Checking if it's a floating-base robot
	bool is_floating_base = false;
	int i = 1;
	while (model.mBodies[i].mIsVirtual) {
		i = model.mu[i][0];
		if (i == 6)
			is_floating_base = true;
	}

	if (is_floating_base) {
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
		base_acc << model.a[6].segment<3>(3) + model.gravity, model.a[6].segment<3>(0);

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
	}
}


} //@namespace rbd
} //@namespace dwl
