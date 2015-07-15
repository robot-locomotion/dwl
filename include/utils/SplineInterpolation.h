#ifndef DWL__MATH__SPLINE_INTERPOLATION__H
#define DWL__MATH__SPLINE_INTERPOLATION__H

#include <stdexcept>


namespace dwl
{

namespace math
{

/**
 * @brief Spline class defines an abstract class for different spline interpolations
 */
class Spline
{
	public:
		struct Point {
			Point() : x(0), xd(0), xdd(0) {}
			Point(double p, double v = 0, double a = 0) : x(p), xd(v), xdd(a) {}
			void setZero() {
				x = 0.0;
				xd = 0.0;
				xdd = 0.0;}
			double x;
			double xd;
			double xdd;
		};
		Spline::Point operator = (const Spline::Point& rhs)
		{
			Spline::Point out;
			out.x = rhs.x;
			out.xd = rhs.xd;
			out.xdd = rhs.xdd;
			return out;
		}

		/** @brief Constructor function */
		Spline();

		/**
		 * @brief Constructor function
		 * @param double Initial time
		 * @param double Duration of the spline
		 * @param const Point& Start point
		 * @param const Point& End point
		 */
		Spline(double initial_time, double duration, const Point& start, const Point& end);

		/** @ Destructor function */
		virtual ~Spline() = 0;

		/**
		 * @brief Sets the boundary of the spline
		 * @param double Initial time
		 * @param double Duration of the spline
		 * @param const Point& Start point
		 * @param const Point& End point
		 */
		void setBoundary(double initial_time, double duration, const Point& start_p, const Point& end_p);

		/**
		 * @brief Sets the boundary of the spline
		 * @param double Initial time
		 * @param double Duration of the spline
		 * @param const double& Start point
		 * @param const double& End point
		 */
		void setBoundary(double initial_time, double duration, const double& start_p, const double& end_p);

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param Point& Point value
		 */
		virtual bool getPoint(const double& current_time, Point& p) = 0;

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param double& Point value
		 */
		virtual bool getPoint(const double& current_time, double& p) = 0;


	protected:
		/** @brief Initial time of the spline */
		double initial_time_;

		/** @brief Duration of the spline */
		double duration_;

		/** Start point of the spline */
		Point start_;

		/** @brief End point of the spline */
		Point end_;
};


/**
 * @brief CubicSpline class defines a cubic spline interpolation
 */
class CubicSpline : public Spline
{
	public:
		/** @brief Constructor function */
		CubicSpline();

		/**
		 * @brief Constructor function
		 * @param double Initial time
		 * @param double Duration of the spline
		 * @param const Point& Start point
		 * @param const Point& End point
		 */
		CubicSpline(double initial_time, double duration, const Point& start, const Point& end);

		/** @ Destructor function */
		~CubicSpline();

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param Point& Point value
		 */
		bool getPoint(const double& current_time, Point& p);

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param double& Point value
		 */
		bool getPoint(const double& current_time, double& p);
};


/**
 * @brief FifthOrderPolySpline class defines a 5-order spline interpolation
 */
class FifthOrderPolySpline : public Spline
{
	public:
		/** @brief Constructor function */
		FifthOrderPolySpline();

		/**
		 * @brief Constructor function
		 * @param double Initial time
		 * @param double Duration of the spline
		 * @param const Point& Start point
		 * @param const Point& End point
		 */
		FifthOrderPolySpline(double initial_time, double duration, const Point& start, const Point& end);

		/** @ Destructor function */
		~FifthOrderPolySpline();

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param Point& Point value
		 */
		bool getPoint(const double& current_time, Point& p);

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param double& Point value
		 */
		bool getPoint(const double& current_time, double& p);
};


/**
 * @brief LinearSpline class defines a linear spline interpolation
 */
class LinearSpline : public Spline
{
	public:
		/** @brief Constructor function */
		LinearSpline();

		/**
		 * @brief Constructor function
		 * @param double Initial time
		 * @param double Duration of the spline
		 * @param const Point& Start point
		 * @param const Point& End point
		 */
		LinearSpline(double initial_time, double duration, const Point& start, const Point& end);

		/** @ Destructor function */
		~LinearSpline();

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param Point& Point value
		 */
		bool getPoint(const double& current_time, Point& p);

		/**
		 * @brief Gets the value of the point according to the spline interpolation
		 * @param const double& Current time
		 * @param double& Point value
		 */
		bool getPoint(const double& current_time, double& p);
};

} //@namespace utils
} //@namespace dwl

#endif
