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
				 double _margin) : point(_point), vertexes(_vertexes),
						 margin(_margin) {}

	Eigen::Vector3d point;
	std::vector<Eigen::Vector3d> vertexes;
	double margin;
};

class SupportPolygonConstraint : public Constraint<PolygonState>
{
	public:
		SupportPolygonConstraint();
		~SupportPolygonConstraint();

		/**
		 * @brief Computes the constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const PolygonState& Whole-body state
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
