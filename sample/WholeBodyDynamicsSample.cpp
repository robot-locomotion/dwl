#include <dwl/WholeBodyState.h>
#include <dwl/model/WholeBodyDynamics.h>


using namespace std;

int main(int argc, char **argv)
{
	dwl::WholeBodyState ws;
	dwl::model::FloatingBaseSystem fbs;
	dwl::model::WholeBodyKinematics wkin;
	dwl::model::WholeBodyDynamics wdyn;

	// Resetting the system from the hyq urdf file
	std::string urdf_file = DWL_SOURCE_DIR"/models/hyq.urdf";
	std::string yarf_file = DWL_SOURCE_DIR"/models/hyq.yarf";
	fbs.resetFromURDFFile(urdf_file, yarf_file);
	wdyn.reset(fbs, wkin);
	wkin.reset(fbs);

	// Define the DoF after initializing the robot model
	ws.setJointDoF(fbs.getJointDoF());


	// The robot state
	ws.setBaseSE3(dwl::SE3(Eigen::Vector3d(0., 0., 0.),
						   Eigen::Vector3d(0., 0., 0.)));
	ws.setBaseVelocity_W(dwl::Motion(Eigen::Vector3d(0., 0., 0.),
									 Eigen::Vector3d(0., 0., 0.)));
	ws.setBaseAcceleration_W(dwl::Motion(Eigen::Vector3d(0., 0., 0.),
										 Eigen::Vector3d(0., 0., 0.)));
	ws.setJointPosition(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointPosition(-1.5, fbs.getJointId("lf_kfe_joint"));
	ws.setJointPosition(-0.75, fbs.getJointId("lh_hfe_joint"));
	ws.setJointPosition(1.5, fbs.getJointId("lh_kfe_joint"));
	ws.setJointPosition(0.75, fbs.getJointId("rf_hfe_joint"));
	ws.setJointPosition(-1.5, fbs.getJointId("rf_kfe_joint"));
	ws.setJointPosition(-0.75, fbs.getJointId("rh_hfe_joint"));
	ws.setJointPosition(1.5, fbs.getJointId("rh_kfe_joint"));

	ws.setJointVelocity(0.2, fbs.getJointId("lf_haa_joint"));
	ws.setJointVelocity(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointVelocity(1., fbs.getJointId("lf_kfe_joint"));
	ws.setJointAcceleration(0.2, fbs.getJointId("lf_haa_joint"));
	ws.setJointAcceleration(0.75, fbs.getJointId("lf_hfe_joint"));
	ws.setJointAcceleration(1., fbs.getJointId("lf_kfe_joint"));


	// Computing the ID
	dwl::Force base_eff = ws.getBaseWrench_W();
	Eigen::VectorXd joint_eff = ws.getJointEffort();
	dwl::ForceMap grfs;
	grfs["lf_foot"] =
			dwl::Force(Eigen::Vector3d(0., 0., 190.778), Eigen::Vector3d::Zero());
	grfs["rf_foot"] =
			dwl::Force(Eigen::Vector3d(0., 0., 190.778), Eigen::Vector3d::Zero());
	grfs["lh_foot"] =
			dwl::Force(Eigen::Vector3d(0., 0., 190.778), Eigen::Vector3d::Zero());
	grfs["rh_foot"] =
			dwl::Force(Eigen::Vector3d(0., 0., 190.778), Eigen::Vector3d::Zero());
	wdyn.computeInverseDynamics(base_eff, joint_eff,
								ws.getBaseSE3(), ws.getJointPosition(),
								ws.getBaseVelocity_W(), ws.getJointVelocity(),
								ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
								grfs);
	cout << "Inverse dynamics" << endl;
	cout << " base wrench = " << base_eff.data << std::endl;
	cout << " joint forces = " << joint_eff.transpose() << std::endl << std::endl;


	// Computing the constrained inverse dynamics
	Eigen::VectorXd joint_forces = Eigen::VectorXd::Zero(fbs.getJointDoF());
	Eigen::VectorXd joint_acc = Eigen::VectorXd::Zero(fbs.getJointDoF());
	dwl::ForceMap contact_forces;
	ws.setBaseAcceleration_W(
			dwl::Motion(Eigen::Vector3d(1.,0.,0.), Eigen::Vector3d::Zero()));
	wdyn.computeConstrainedInverseDynamics(joint_forces, joint_acc, contact_forces,
										   ws.getBaseSE3(), ws.getJointPosition(),
										   ws.getBaseVelocity_W(), ws.getJointVelocity(),
										   ws.getBaseAcceleration_W(),
										   fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Constrained inverse dynamics" << endl;
	cout << " joint forces = " << joint_forces.transpose() << endl;
	cout << " joint accelerations = " << joint_acc.transpose() << endl;
	cout << " contact forces = " << endl;
	for (dwl::ForceMap::iterator it = contact_forces.begin();
		it != contact_forces.end(); ++it) {
		cout << it->first << ":" << endl;
		cout << it->second.data << endl;
	}
	cout << endl;


	// Computing contact forces and joint accelerations
	wdyn.computeContactForces(contact_forces, joint_acc,
							  ws.getBaseSE3(), ws.getJointPosition(),
							  ws.getBaseVelocity_W(), ws.getJointVelocity(),
							  ws.getBaseAcceleration_W(),
							  fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Contact forces and joint accelerations" << endl;
	cout << " contact forces = " << endl;
	for (dwl::ForceMap::iterator it = contact_forces.begin();
			it != contact_forces.end(); it++) {
		cout << it->first << ":" << endl;
		cout << it->second.data << endl;
	}
	cout << " joint accelerations = " << joint_acc.transpose() << endl << endl;


	// Computing the joint space inertia matrix
	Eigen::MatrixXd M =
			wdyn.computeJointSpaceInertiaMatrix(ws.getBaseSE3(),
												ws.getJointPosition());
	cout << "Joint-space Inertia Matrix" << endl;
	cout << M << endl << endl;


	// Computing the centroidal inertia matrix
	Eigen::Matrix6d H =
			wdyn.computeCentroidalInertiaMatrix(ws.getBaseSE3(),
												ws.getJointPosition(),
												ws.getBaseVelocity_W(),
												ws.getJointVelocity());
	cout << "Centroidal Inertia Matrix" << endl;
	cout << H << endl << endl;


	// Computing the centroidal momentum matrix
	Eigen::Matrix6x I =
			wdyn.computeCentroidalMomentumMatrix(ws.getBaseSE3(),
												 ws.getJointPosition(),
												 ws.getBaseVelocity_W(),
												 ws.getJointVelocity());
	cout << "Centroidal Momentum Matrix" << endl;
	cout << I << endl << endl;



	// Computing the gravito wrench
	dwl::Force grav_wrench =
			wdyn.computeGravitoWrench(ws.getBaseSE3(), ws.getJointPosition());
	cout << "Gravito wrench = " << endl;
	cout << grav_wrench.data << endl << endl;


	// Estimated contact forces
	dwl::ForceMap est_forces;
	wdyn.estimateContactForces(est_forces,
	                           ws.getBaseSE3(), ws.getJointPosition(),
	                           ws.getBaseVelocity_W(), ws.getJointVelocity(),
	                           ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
	                           ws.getJointEffort(), fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Estimated contact forces = " << endl;
	for (dwl::ForceMap::iterator it = est_forces.begin();
			it != est_forces.end(); it++) {
		cout << it->first << ":" << endl;
		cout << it->second.data << endl;
	}
	cout << endl;


	// Estimated GRFs from CoP
	Eigen::Vector3d cop_pos(0.,0.,-0.58);
	dwl::SE3Map contact_pos =
			wkin.computePosition(ws.getBaseSE3(), ws.getJointPosition(),
								fbs.getEndEffectorList(dwl::model::FOOT));
	wdyn.estimateGroundReactionForces(grfs,
	                                  cop_pos, contact_pos,
	                                  fbs.getEndEffectorList(dwl::model::FOOT));
	cout << "Estimated GRFs from CoP = " << endl;
	for (dwl::ForceMap::iterator it = grfs.begin();
			it != grfs.end(); it++) {
		cout << it->first << ":" << endl;
		cout << it->second.data << endl;
	}
	cout << endl;


	// Estimating active contacts
	double force_threshold = 50.;
	dwl::model::ElementList active_contacts;
	wdyn.estimateActiveContactsAndForces(active_contacts, est_forces,
										 ws.getBaseSE3(), ws.getJointPosition(),
	                                     ws.getBaseVelocity_W(), ws.getJointVelocity(),
	                                     ws.getBaseAcceleration_W(), ws.getJointAcceleration(),
	                                     ws.getJointEffort(), fbs.getEndEffectorList(dwl::model::FOOT), // it uses all the end-effector of the system
										 force_threshold);
	cout << "Estimated active contacts" << endl;
	for (dwl::ForceMap::iterator it = est_forces.begin();
			it != est_forces.end(); it++) {
		cout << it->first << ":" << endl;
		cout << it->second.data << endl;
	}
	cout << endl;


	// Getting active contacts
	est_forces["lf_foot"] = dwl::Force();
	wdyn.getActiveContacts(active_contacts,
	                       est_forces, force_threshold);
	cout << "Active contacts = " << endl;
	for (dwl::model::ElementList::iterator it = active_contacts.begin();
			it != active_contacts.end(); it++) {
		cout << *it << ", ";
	}
	cout << endl;



	// Computing the center of pressure
	dwl::Force f(Eigen::Vector3d(0., 0., 190.778),
				 Eigen::Vector3d::Zero());
	contact_forces["lf_foot"] = f;
	contact_forces["lh_foot"] = f;
	contact_forces["rf_foot"] = f;
	contact_forces["rh_foot"] = f;
	wdyn.computeCenterOfPressure(cop_pos,
	                             contact_forces,
	                             contact_pos);
	cout << "Center of Pressure =" << endl;
	cout << cop_pos.transpose() << endl << endl;


	// Computing the zero moment point
	Eigen::Vector3d zmp_pos;
	Eigen::Vector3d c_pos, c_vel, c_acc;
	double height = 0.589;
	wkin.computeCoMRate(c_pos, c_vel, c_acc,
	                    ws.getBaseSE3(), ws.getJointPosition(),
	                    ws.getBaseVelocity_W(), ws.getJointVelocity(),
	                    ws.getBaseAcceleration_W(), ws.getJointAcceleration());
	wdyn.computeZeroMomentPoint(zmp_pos,
								c_pos, c_acc, height);
	cout << "Zero Moment Point =" << endl;
	cout << zmp_pos.transpose() << endl << endl;


	// Computing the instantaneous capture point
	Eigen::Vector3d icp_pos;
	wdyn.computeInstantaneousCapturePoint(icp_pos, c_pos, c_vel, height);
	cout << "Instantaneous Capture Point" << endl;
	cout << icp_pos.transpose() << endl << endl;


	// Computing the centroidal moment pivot
	Eigen::Vector3d cmp_pos;
	wdyn.computeCentroidalMomentPivot(cmp_pos, c_pos, height, contact_forces);
	cout << "Centroidal Moment Pivot" << endl;
	cout << cmp_pos.transpose() << endl << endl;


	// Computing the CoM torque
	Eigen::Vector3d com_torque;
	wdyn.computeCoMTorque(com_torque,
	                      c_pos, cmp_pos,
	                      contact_forces);
	cout << "CoM torque" << endl;
	cout << com_torque.transpose() << endl << endl;

	return 0;
}
