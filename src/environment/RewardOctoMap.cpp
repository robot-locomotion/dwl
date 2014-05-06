#include <environment/RewardOctoMap.h>


namespace dwl
{

namespace environment
{


RewardOctoMap::RewardOctoMap()
{

}


RewardOctoMap::~RewardOctoMap()
{

}


void RewardOctoMap::compute(Modeler model)
{
	octomap::OcTree* octomap = model.octomap;

	// To ensure memory reallocation
	std::vector<Pose> empty;
	normals_.swap(empty);

	bool is_there_neighboring = false;
	for (double y = search_area_.min_y; y <= search_area_.max_y; y += search_area_.grid_size) {
		for (double x = search_area_.min_x; x <= search_area_.max_x; x += search_area_.grid_size) {
			double z = search_area_.max_z;
			int r = 0;
			bool is_found_surface = false;

			octomap::OcTreeKey init_key;
			if (!octomap->coordToKeyChecked(x, y, z, 16, init_key)) {
				printf(RED "Voxel out of bounds" COLOR_RESET);

				return;
			}

			// Finding the voxel of the surface
			while (!is_found_surface && z >= search_area_.min_z) {
				octomap::OcTreeKey heightmap_key;
				octomap::OcTreeNode* heightmap_node = octomap->search(init_key);
				heightmap_key[0] = init_key[0];
				heightmap_key[1] = init_key[1];
				heightmap_key[2] = init_key[2] - r;

				heightmap_node = octomap->search(heightmap_key);
				octomap::point3d height_point = octomap->keyToCoord(heightmap_key);
				double xh = height_point(0);
				double yh = height_point(1);
				z = height_point(2);

				if (heightmap_node) {
					if (octomap->isNodeOccupied(heightmap_node)) {
						std::vector<octomap::point3d> points;
						octomap::OcTreeKey current_key;
						octomap::OcTreeNode* current_node = heightmap_node;

						// Iterate over the 8 neighboring sets
						int minX = -1; int maxX = 1;
						int minY = -1; int maxY = 1;
						int minZ = -1; int maxZ = 1;
						for (int i = minZ; i < maxZ + 1; i++) {
							for (int j = minY; j < maxY + 1; j++) {
								for (int k = minX; k < maxX + 1; k++) {
									current_key[0] = heightmap_key[0] + k;
									current_key[1] = heightmap_key[1] + j;
									current_key[2] = heightmap_key[2] + i;
									current_node = octomap->search(current_key);
									if (current_node) {
										if (octomap->isNodeOccupied(current_node)) {
											octomap::point3d point = octomap->keyToCoord(current_key);
											points.push_back(point);

											is_there_neighboring = true;
										}
									}
								}
							}
						}
						if (is_there_neighboring) {
							Eigen::Vector3d surface_normal;
							float curvature;
							if (computeRewards(points, surface_normal, curvature)) {

							}
							is_there_neighboring = false;
						}

						is_found_surface = true;
					}

				}
				r++;
			}
		}
	}
}


bool RewardOctoMap::computeRewards(std::vector<octomap::point3d> cloud, Eigen::Vector3d &surface_normal, float &curvature)
{
	EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
	Eigen::Vector3d mean;
	if (cloud.size() < 3 || computeMeanAndCovarianceMatrix(cloud, covariance_matrix, mean) == 0) {
		surface_normal = Eigen::Vector3d::Zero();
		curvature = 0;

		return false;
	}

	solvePlaneParameters(covariance_matrix, surface_normal, curvature);

	double slope = fabs(acos((double) surface_normal(2)));
	std::cout << "slope = " << slope << std::endl;


	// record data
	Eigen::Vector3d origin_vector = Eigen::Vector3d::Zero();
	origin_vector(0) = 1;
	Eigen::Quaternion<double> orientation;
	orientation.setFromTwoVectors(origin_vector, surface_normal);

	Pose pose;
	pose.position = mean;
	pose.orientation(0) = orientation.x();
	pose.orientation(1) = orientation.y();
	pose.orientation(2) = orientation.z();
	pose.orientation(3) = orientation.w();
	normals_.push_back(pose);


	Terrain terrain_info;
	terrain_info.position = mean;
	terrain_info.surface_normal = surface_normal;
	terrain_info.curvature = curvature;

	if (is_added_feature_) {
		double reward_value;
		for (int i = 0; i < features_.size(); i++) {
			features_[i]->computeReward(reward_value, terrain_info);
			std::cout << features_[i]->getName() << " value = " << reward_value << std::endl;
		}
	}
	else
		printf(YELLOW "Could not compute the reward of the features because it is necessary to add ones\n" COLOR_RESET);


	return true;
}


unsigned int RewardOctoMap::computeMeanAndCovarianceMatrix(std::vector<octomap::point3d> cloud, Eigen::Matrix3d &covariance_matrix, Eigen::Vector3d &mean)
{
	// create the buffer on the stack which is much faster than using cloud[indices[i]] and mean as a buffer
	Eigen::VectorXd accu = Eigen::VectorXd::Zero(9, 1);

	for (size_t i = 0; i < cloud.size(); ++i) {
		accu[0] += cloud[i](0) * cloud[i](0);
		accu[1] += cloud[i](0) * cloud[i](1);
		accu[2] += cloud[i](0) * cloud[i](2);
		accu[3] += cloud[i](1) * cloud[i](1); // 4
		accu[4] += cloud[i](1) * cloud[i](2); // 5
		accu[5] += cloud[i](2) * cloud[i](2); // 8
		accu[6] += cloud[i](0);
		accu[7] += cloud[i](1);
		accu[8] += cloud[i](2);
	}

	accu /= (cloud.size());
	if (cloud.size() != 0) {
		mean[0] = accu[6];
		mean[1] = accu[7];
		mean[2] = accu[8];

		covariance_matrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
		covariance_matrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
		covariance_matrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
		covariance_matrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
		covariance_matrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
		covariance_matrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
		covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
		covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
		covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
	}

	return (static_cast<unsigned int> (cloud.size()));
}


void RewardOctoMap::solvePlaneParameters(const Eigen::Matrix3d &covariance_matrix, Eigen::Vector3d &normal_vector, float &curvature)
{
	// Extract the smallest eigenvalue and its eigenvector
	EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigenvalue;
	EIGEN_ALIGN16 Eigen::Vector3d eigenvector;

	//typedef typename Eigen::Matrix3f::Scalar Scalar;
	// Scale the matrix so its entries are in [-1,1]. The scaling is applied
	// only when at least one matrix entry has magnitude larger than 1.
	Eigen::Matrix3d::Scalar scale = covariance_matrix.cwiseAbs().maxCoeff();
	if (scale <= std::numeric_limits<Eigen::Matrix3d::Scalar>::min())
		scale = Eigen::Matrix3d::Scalar(1.0);

	Eigen::Matrix3d scaledMat = covariance_matrix / scale;

	Eigen::Vector3d eigenvalues;
	computeRoots(scaledMat, eigenvalues);

	eigenvalue = eigenvalues(0) * scale;

	scaledMat.diagonal().array() -= eigenvalues(0);

	Eigen::Vector3d vec1 = scaledMat.row(0).cross(scaledMat.row(1));
	Eigen::Vector3d vec2 = scaledMat.row(0).cross(scaledMat.row(2));
	Eigen::Vector3d vec3 = scaledMat.row(1).cross(scaledMat.row(2));

	Eigen::Matrix3d::Scalar len1 = vec1.squaredNorm();
	Eigen::Matrix3d::Scalar len2 = vec2.squaredNorm();
	Eigen::Matrix3d::Scalar len3 = vec3.squaredNorm();

	if (len1 >= len2 && len1 >= len3)
		eigenvector = vec1 / std::sqrt(len1);
	else if (len2 >= len1 && len2 >= len3)
		eigenvector = vec2 / std::sqrt(len2);
	else
		eigenvector = vec3 / std::sqrt(len3);


	normal_vector = eigenvector;

	// Compute the curvature surface change
	float eig_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);
	if (eig_sum != 0)
		curvature = fabsf(eigenvalue / eig_sum);
	else
		curvature = 0;
}


void RewardOctoMap::computeRoots(const Eigen::Matrix3d& m, Eigen::Vector3d& roots)
{
	// The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0. The
	// eigenvalues are the roots to this equation, all guaranteed to be
	// real-valued, because the matrix is symmetric.
	Eigen::Matrix3d::Scalar c0 = m (0, 0) * m (1, 1) * m (2, 2) + Eigen::Matrix3d::Scalar (2) * m (0, 1) * m (0, 2) * m (1, 2)
	- m (0, 0) * m (1, 2) * m (1, 2) - m (1, 1) * m (0, 2) * m (0, 2)
	- m (2, 2) * m (0, 1) * m (0, 1);

	Eigen::Matrix3d::Scalar c1 = m (0, 0) * m (1, 1) - m (0, 1) * m (0, 1) + m (0, 0) * m (2, 2) -
	m (0, 2) * m (0, 2) + m (1, 1) * m (2, 2) - m (1, 2) * m (1, 2);

	Eigen::Matrix3d::Scalar c2 = m (0, 0) + m (1, 1) + m (2, 2);


	if (fabs (c0) < Eigen::NumTraits<Eigen::Matrix3d::Scalar>::epsilon ())// one root is 0 -> quadratic equation
		computeRoots2(c2, c1, roots);
	else
	{
		const Eigen::Matrix3d::Scalar s_inv3 = Eigen::Matrix3d::Scalar(1.0 / 3.0);
		const Eigen::Matrix3d::Scalar s_sqrt3 = std::sqrt(Eigen::Matrix3d::Scalar (3.0));
		// Construct the parameters used in classifying the roots of the equation
		// and in solving the equation for the roots in closed form.
		Eigen::Matrix3d::Scalar c2_over_3 = c2*s_inv3;
		Eigen::Matrix3d::Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
		if (a_over_3 > Eigen::Matrix3d::Scalar(0))
			a_over_3 = Eigen::Matrix3d::Scalar (0);

		Eigen::Matrix3d::Scalar half_b = Eigen::Matrix3d::Scalar(0.5) * (c0 + c2_over_3 * (Eigen::Matrix3d::Scalar(2) * c2_over_3 * c2_over_3 - c1));

		Eigen::Matrix3d::Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
		if (q > Eigen::Matrix3d::Scalar(0))
			q = Eigen::Matrix3d::Scalar(0);

		// Compute the eigenvalues by solving for the roots of the polynomial.
		Eigen::Matrix3d::Scalar rho = sqrt(-a_over_3);
		Eigen::Matrix3d::Scalar theta = atan2(std::sqrt (-q), half_b) * s_inv3;
		Eigen::Matrix3d::Scalar cos_theta = cos(theta);
		Eigen::Matrix3d::Scalar sin_theta = sin(theta);
		roots(0) = c2_over_3 + Eigen::Matrix3d::Scalar(2) * rho * cos_theta;
		roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
		roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

		// Sort in increasing order.
		double roots0 = roots(0);
		double roots1 = roots(1);
		double roots2 = roots(2);
		if (roots(0) >= roots(1))
			std::swap(roots0, roots1);
		if (roots(1) >= roots(2)) {
			std::swap(roots1, roots2);
			if (roots(0) >= roots(1))
				std::swap(roots0, roots1);
		}
		roots(0) = roots0;
		roots(1) = roots1;
		roots(2) = roots2;

		if (roots(0) <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
			computeRoots2(c2, c1, roots);
	}
}


void RewardOctoMap::computeRoots2(const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c, Eigen::Vector3d& roots)
{
	roots(0) = Eigen::Matrix3d::Scalar(0);
	Eigen::Matrix3d::Scalar d = Eigen::Matrix3d::Scalar(b * b - 4.0 * c);
	if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
		d = 0.0;

	Eigen::Matrix3d::Scalar sd = sqrt(d);

	roots(2) = 0.5f * (b + sd);
	roots(1) = 0.5f * (b - sd);
}


std::vector<Pose> RewardOctoMap::getNormals()
{
	return normals_;
}


} //@namespace environment

} //@namespace dwl
