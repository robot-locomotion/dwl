#ifndef DWL_SplineInterpolation_H
#define DWL_SplineInterpolation_H

#include <stdexcept>


namespace dwl
{

namespace utils
{


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

		Spline();
		Spline(double duration, const Point& start, const Point& end);
		virtual ~Spline() = 0;
		void setBoundary(double T, const Point& start_p, const Point& end_p);
		void setBoundary(double T, const double& start_p, const double& end_p);

		virtual bool getPoint(const double& dt, Point& p) = 0;
		virtual bool getPoint(const double& dt, double& p) = 0;


	protected:
		double duration;
		Point start;
		Point end;
};


class CubicSpline : public Spline
{
	public:
		CubicSpline();
		CubicSpline(double duration, const Point& start, const Point& end);
		~CubicSpline();

		bool getPoint(const double& dt, Point& p);
		bool getPoint(const double& dt, double& p);
};



class FifthOrderPolySpline : public Spline
{
	public:
		FifthOrderPolySpline();
		FifthOrderPolySpline(double duration, const Point& start, const Point& end);
		~FifthOrderPolySpline();

		bool getPoint(const double& dt, Point& p);
		bool getPoint(const double& dt, double& p);
};



class LinearSpline : public Spline
{
	public:
		LinearSpline();
		LinearSpline(double duration, const Point& start, const Point& end);
		~LinearSpline();

		bool getPoint(const double& dt, Point& p);
		bool getPoint(const double& dt, double& p);
};

} //@namespace utils
} //@namespace dwl

#endif
