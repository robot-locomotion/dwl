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

Part3d linearFloatingBaseState(Vector6d& vector)
{
	return vector.topRows<3>();
}


Part3d angularFloatingBaseState(Vector6d& vector)
{
	return vector.bottomRows<3>();
}


bool isFloatingBaseRobot(const RigidBodyDynamics::Model& model)
{
	bool is_floating_base = false;
	int i = 1;
	while (model.mBodies[i].mIsVirtual) {
		i = model.mu[i][0];
		if (i == 6)
			is_floating_base = true;
	}

	return is_floating_base;
}


unsigned int getFloatingBaseDOF(const RigidBodyDynamics::Model& model,
								struct rbd::ReducedFloatingBase* reduced_base)
{
	unsigned int floating_base_dof = 0;
	if (isFloatingBaseRobot(model))
		floating_base_dof = 6;
	else if (reduced_base != NULL)
		floating_base_dof = reduced_base->getFloatingBaseDOF();

	return floating_base_dof;
}


Eigen::VectorXd toGeneralizedJointState(const RigidBodyDynamics::Model& model,
										const Vector6d& base_state,
										const Eigen::VectorXd& joint_state,
										struct rbd::ReducedFloatingBase* reduced_base)
{
	// Note that RBDL defines the floating base state as [linear states, angular states]
	Eigen::VectorXd q(model.dof_count);
	if (isFloatingBaseRobot(model))
		q << base_state, joint_state;
	else if (reduced_base != NULL) {
		Eigen::VectorXd virtual_base(reduced_base->getFloatingBaseDOF());
		if (reduced_base->TX.active)
			virtual_base(reduced_base->TX.id) = base_state(TX);
		if (reduced_base->TY.active)
			virtual_base(reduced_base->TY.id) = base_state(TY);
		if (reduced_base->TZ.active)
			virtual_base(reduced_base->TZ.id) = base_state(TZ);
		if (reduced_base->RX.active)
			virtual_base(reduced_base->RX.id) = base_state(RX);
		if (reduced_base->RY.active)
			virtual_base(reduced_base->RY.id) = base_state(RY);
		if (reduced_base->RZ.active)
			virtual_base(reduced_base->RZ.id) = base_state(RZ);

		q << virtual_base, joint_state;
	} else
		q = joint_state;

	return q;
}


void fromGeneralizedJointState(const RigidBodyDynamics::Model& model,
							   Vector6d& base_state,
							   Eigen::VectorXd& joint_state,
							   const Eigen::VectorXd& generalized_state,
							   struct rbd::ReducedFloatingBase* reduced_base)
{
	// Note that RBDL defines the floating base state as [linear states, angular states]
	if (isFloatingBaseRobot(model)) {
		base_state = generalized_state;
		joint_state = generalized_state.tail(model.dof_count - 6);
	} else if (reduced_base != NULL) {
		if (reduced_base->TX.active)
			base_state(TX) = generalized_state(reduced_base->TX.id);
		if (reduced_base->TY.active)
			base_state(TY) = generalized_state(reduced_base->TY.id);
		if (reduced_base->TZ.active)
			base_state(TZ) = generalized_state(reduced_base->TZ.id);
		if (reduced_base->RX.active)
			base_state(RX) = generalized_state(reduced_base->RX.id);
		if (reduced_base->RY.active)
			base_state(RY) = generalized_state(reduced_base->RY.id);
		if (reduced_base->RZ.active)
			base_state(RZ) = generalized_state(reduced_base->RZ.id);

		joint_state = generalized_state.tail(model.dof_count - reduced_base->getFloatingBaseDOF());
	} else {
		base_state = Vector6d::Zero();
		joint_state = generalized_state;
	}
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
			G.block(0, q_index, 6, 3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]);
		} else {
			G.block(0,q_index, 6, 1) = point_trans.apply(model.X_base[j].inverse().apply(model.S[j]));
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

	if (isFloatingBaseRobot(model)) {
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
}


} //@namespace rbd
} //@namespace dwl
