#include <dwl/ocp/SupportPolygonConstraint.h>

#define BOOST_TEST_MODULE DWL_TESTS
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>


struct PointTest
{
	PointTest(const Eigen::Vector3d& _point,
			  bool _inside) : point(_point), inside(_inside) {}

	Eigen::Vector3d point;
	bool inside;
};


BOOST_AUTO_TEST_CASE(squared_support) // specify a test case for squared region
{
	// Declaring the constraint
	dwl::ocp::SupportPolygonConstraint constraint;
	constraint.setSoftProperties(dwl::ocp::SoftConstraintProperties(10, 0.));

	// Defining the squared support region
	double dim = 1.;
	std::vector<Eigen::Vector3d> support;
	support.push_back(Eigen::Vector3d(0., 0., 0.));
	support.push_back(Eigen::Vector3d(dim, 0., 0.));
	support.push_back(Eigen::Vector3d(0., dim, 0.));
	support.push_back(Eigen::Vector3d(dim, dim, 0.));

	// Defining the support margin
	double margin = 0.05;

	// Defining different test points
	std::vector<PointTest> test_set;
	test_set.push_back(PointTest(Eigen::Vector3d(margin, margin, 0.), true));
	test_set.push_back(PointTest(Eigen::Vector3d(dim + margin, dim + margin, 0.), false));
	test_set.push_back(PointTest(Eigen::Vector3d(dim - margin, dim - margin, 0.), true));
	test_set.push_back(PointTest(Eigen::Vector3d(dim - 0.5*margin, dim - 0.5*margin, 0.), false));

	// Computing the constraints values for the defined points
	for (unsigned int k = 0; k < test_set.size(); k++) {
		// Getting the state
		dwl::ocp::PolygonState state(test_set[k].point, support, margin);

		// Computing the constraint
		Eigen::VectorXd value;
		double cost;
		constraint.compute(value, state);
		constraint.computeSoft(cost, state);

		// Getting the lower value for comparison
		double lower_value = std::numeric_limits<double>::max();
		for (unsigned int k = 0; k < value.size(); k++) {
			if (value(k) < lower_value) {
				lower_value = value(k);
			}
		}

		if (test_set[k].inside) {
			BOOST_REQUIRE_MESSAGE(lower_value >= 0.,
								  "The [" << test_set[k].point.transpose() << "]^T point is not inside");
			BOOST_REQUIRE_MESSAGE(cost == 0.,
								  "The cost value (" << cost << ") is not equals to 0");
		} else {
			BOOST_REQUIRE_MESSAGE(lower_value <= 0.,
								  "The [" << test_set[k].point.transpose() << "]^T point is not outside");
			BOOST_REQUIRE_MESSAGE(cost > 0.,
								  "The cost value (" << cost << ") is not positive");
		}
	}
}


BOOST_AUTO_TEST_CASE(triangle_support) // specify a test case for triangle region
{
	// Declaring the constraint
	dwl::ocp::SupportPolygonConstraint constraint;
	constraint.setSoftProperties(dwl::ocp::SoftConstraintProperties(10, 0.));

	// Defining the squared support region
	double dim = 1.;
	std::vector<Eigen::Vector3d> support;
	support.push_back(Eigen::Vector3d(0., 0., 0.));
	support.push_back(Eigen::Vector3d(dim, 0., 0.));
	support.push_back(Eigen::Vector3d(0., dim, 0.));

	// Defining the support margin
	double margin = 0.05;

	// Defining different test points
	std::vector<PointTest> test_set;
	test_set.push_back(PointTest(Eigen::Vector3d(margin, margin, 0.), true));
	test_set.push_back(PointTest(Eigen::Vector3d(0.5 * dim + margin, 0.5 * dim + margin, 0.), false));
	test_set.push_back(PointTest(Eigen::Vector3d(0.5 * dim - margin, 0.5 * dim - margin, 0.), true));
	test_set.push_back(PointTest(Eigen::Vector3d(0.5 * (dim + margin), 0.5 * (dim + margin), 0.), false));

	// Computing the constraints values for the defined points
	for (unsigned int k = 0; k < test_set.size(); k++) {
		// Getting the state
		dwl::ocp::PolygonState state(test_set[k].point, support, margin);

		// Computing the constraint
		Eigen::VectorXd value;
		double cost;
		constraint.compute(value, state);
		constraint.computeSoft(cost, state);

		// Getting the lower value for comparison
		double lower_value = std::numeric_limits<double>::max();
		for (unsigned int k = 0; k < value.size(); k++) {
			if (value(k) < lower_value) {
				lower_value = value(k);
			}
		}

		if (test_set[k].inside) {
			BOOST_REQUIRE_MESSAGE(lower_value >= 0.,
								  "The [" << test_set[k].point.transpose() << "]^T point is not inside");
			BOOST_REQUIRE_MESSAGE(cost == 0.,
								  "The cost value (" << cost << ") is not equals to 0");
		} else {
			BOOST_REQUIRE_MESSAGE(lower_value <= 0.,
								  "The [" << test_set[k].point.transpose() << "]^T point is not outside");
			BOOST_REQUIRE_MESSAGE(cost > 0.,
								  "The cost value (" << cost << ") is not positive");
		}
	}
}

BOOST_AUTO_TEST_CASE(line_support) // specify a test case for line region
{
	// Declaring the constraint
	dwl::ocp::SupportPolygonConstraint constraint;
	constraint.setSoftProperties(dwl::ocp::SoftConstraintProperties(10, 0.));

	// Defining the squared support region
	double dim = 1.;
	std::vector<Eigen::Vector3d> support;
	support.push_back(Eigen::Vector3d(0., 0., 0.));
	support.push_back(Eigen::Vector3d(dim, dim, 0.));

	// Defining the support margin
	double margin = 0.05;

	// Defining different test points
	std::vector<PointTest> test_set;
	test_set.push_back(PointTest(Eigen::Vector3d(0., 0., 0.), true));
	test_set.push_back(PointTest(Eigen::Vector3d(1.2 * dim, 1.2 * dim, 0.), false));
	test_set.push_back(PointTest(Eigen::Vector3d(0.5 * dim, 0.5 * dim, 0.), true));
	test_set.push_back(PointTest(Eigen::Vector3d(-1.2, -1.2, 0.), false));

	// Computing the constraints values for the defined points
	for (unsigned int k = 0; k < test_set.size(); k++) {
		// Getting the state
		dwl::ocp::PolygonState state(test_set[k].point, support, margin);

		// Computing the constraint
		Eigen::VectorXd value;
		double cost;
		constraint.compute(value, state);
		constraint.computeSoft(cost, state);

		// Getting the lower value for comparison
		double lower_value = std::numeric_limits<double>::max();
		for (unsigned int k = 0; k < value.size(); k++) {
			if (value(k) < lower_value) {
				lower_value = value(k);
			}
		}

		if (test_set[k].inside) {
			BOOST_REQUIRE_MESSAGE(lower_value >= 0.,
								  "The [" << test_set[k].point.transpose() << "]^T point is not inside");
			BOOST_REQUIRE_MESSAGE(cost == 0.,
								  "The cost value (" << cost << ") is not equals to 0");
		} else {
			BOOST_REQUIRE_MESSAGE(lower_value <= 0.,
								  "The [" << test_set[k].point.transpose() << "]^T point is not outside");
			BOOST_REQUIRE_MESSAGE(cost >= 0.,
								  "The cost value (" << cost << ") is not positive");
		}
	}
}
