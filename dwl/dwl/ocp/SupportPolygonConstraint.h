#ifndef DWL__OCP__SUPPORT_POLYGON_CONSTRAINT__H
#define DWL__OCP__SUPPORT_POLYGON_CONSTRAINT__H

#include <dwl/ocp/Constraint.h>


namespace dwl
{

namespace ocp
{

struct PolygonState
{
	PolygonState(Eigen::Vector3d& _point,
				 std::vector<Eigen::Vector3d> _vertexes,
				 double _margin) : point(_point),
						 vertexes(_vertexes.begin(), _vertexes.end()),
						 margin(_margin) {}

	Eigen::Vector3d point;
	std::vector<Eigen::Vector3d> vertexes;
	double margin;
};

/**
 * @class SupportPolygonConstraint
 * @brief This is specialization class for imposing polygonal
 * constraints, i.e. a 2d point inside the defined polygon. This
 * method could be used for imposing static and dynamic stabilities
 * constraints
 */
class SupportPolygonConstraint : public Constraint<PolygonState>
{
	public:
		/** @brief Constructor function */
		SupportPolygonConstraint();

		/** @brief Destructor function */
		~SupportPolygonConstraint();

		/**
		 * @brief Computes the constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const PolygonState& Polygon state
		 */
		void compute(Eigen::VectorXd& constraint,
					 const PolygonState& state);

		/**
		 * @brief Gets the lower and upper bounds of the constraint
		 * @param Eigen::VectorXd& Lower constraint bound
		 * @param Eigen::VectorXd& Upper constraint bound
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);


	private:
		/** @brief Number of polygon lines */
		unsigned int num_lines_;
};
} //@namespace ocp
} //@namespace dwl

#endif
