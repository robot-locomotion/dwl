#include <utils/SplineInterpolation.h>


namespace dwl
{


namespace utils
{

inline Spline::Spline()
{

}


inline Spline::Spline(double T, const Point& start_p, const Point& end_p) :
	duration(T), start(start_p), end(end_p)
{
	if(duration <= 0)
		throw std::invalid_argument("Cannot create a Spliner with zero or negative duration");
}


inline Spline::~Spline()
{

}


inline void Spline::setBoundary(double T, const Point& start_p, const Point& end_p)
{
	 duration = T;
	 start = start_p;
	 end = end_p;
}


inline void Spline::setBoundary(double T, const double& start_p, const double& end_p)
{
	 duration = T;

	 start.x = start_p;
	 start.xd = 0.0;
	 start.xdd = 0.0;

	 end.x = end_p;
	 end.xd = 0.0;
	 end.xdd = 0.0;
}


inline CubicSpline::CubicSpline(double duration, const Point& start, const Point& end) :
	Spline(duration, start, end)
{

}


inline CubicSpline::CubicSpline()
{

}

CubicSpline::~CubicSpline()
{

}


bool CubicSpline::getPoint(const double& user_dt, Point& out)
{
    double dt = user_dt;
    if (dt > duration ) {
        dt = duration;
    }

    // sanity checks
    if (dt < 0) {
        return false;
    }

    double a0,a1,a2,a3;
    double T1,T2,T3;
    double dt2,dt3;

    // powers of the duration
    T1 = duration;
    T2 = duration * T1;
    T3 = duration * T2;

    // powers of dt
    dt2 = dt*dt;
    dt3 = dt*dt2;

    // spline coefficients
    a0 = start.x;
    a1 = start.xd;
    a2 = -((3 * a0) - (3 * end.x) + (2 * T1 * a1) + (T1 * end.xd)) / T2;
    a3 = ((2 * a0) - (2 * end.x) + T1 * (a1 + end.xd)) / T3;

    // interpolated point
    out.x = a0 + a1 * dt + a2 * dt2 + a3 * dt3;
    out.xd = a1 + 2 * a2 * dt + 3 * a3 * dt2;
    out.xdd = 2*a2 + 6 * a3 * dt;

    return true;
}


bool CubicSpline::getPoint(const double& user_dt, double& pos)
{
	Point out;
	CubicSpline::getPoint(user_dt, out);
	pos = out.x;
	return true;
}


inline FifthOrderPolySpline::FifthOrderPolySpline()
{

}


inline FifthOrderPolySpline::FifthOrderPolySpline(double duration, const Point& start, const Point& end) :
	Spline(duration, start, end)
{

}

FifthOrderPolySpline::~FifthOrderPolySpline()
{

}


bool FifthOrderPolySpline::getPoint(const double& user_dt, Point& out)
{
    double dt = user_dt;
    if (dt > duration ) {
        dt = duration;
    }

    // sanity checks
    if (dt < 0) {
        return false;
    }

    double a0, a1, a2, a3, a4, a5;
    double T1, T2, T3, T4, T5;
    double dt2, dt3, dt4, dt5;

    // powers of duration
    T1 = duration;
    T2 = duration * duration;
    T3 = duration * T2;
    T4 = duration * T3;
    T5 = duration * T4;

    dt2 = dt * dt;
    dt3 = dt * dt2;
    dt4 = dt * dt3;
    dt5 = dt * dt4;

    // spline coefficients
    a0 = start.x;
    a1 = start.xd;
    a2 = start.xdd / 2;
    a3 = (-20 * a0 + 20 * end.x + T1 * (-3 * a2 / 2 * T1 + end.xdd * T1 - 12 * a1 - 8 * end.xd)) / (2 * T3);
    a4 = (30 * a0 - 30 * end.x + T1 * (3 * start.xdd * T1 - 2 * end.xdd * T1 + 16 * start.xd + 14 * end.xd)) / (2 * T4);
    a5 = -(12 * start.x - 12 * end.x + T1 * (start.xdd * T1 - end.xdd * T1 + 6 * (start.xd + end.xd))) / (2 * T5);

    out.x = a0 + a1 * dt + a2 * dt2 + a3 * dt3 + a4 * dt4 + a5 * dt5;
    out.xd = a1 + 2 * a2 * dt + 3 * a3 * dt2 + 4 * a4 * dt3 + 5 * a5 * dt4;
    out.xdd = a2 + 6 * a3 * dt + 12 * a4 * dt2 + 20 * a5 * dt3;

    return true;
}


bool FifthOrderPolySpline::getPoint(const double& user_dt, double& pos)
{
	Point out;
	FifthOrderPolySpline::getPoint(user_dt, out);
	pos = out.x;

	return true;
}


inline LinearSpline::LinearSpline()
{

}


inline LinearSpline::LinearSpline(double duration, const Point& start, const Point& end) :
	Spline(duration, start, end)
{

}


LinearSpline::~LinearSpline()
{

}


bool LinearSpline::getPoint(const double& user_dt, Point& out)
{
	double dt = user_dt;
	if (dt > duration )
		dt = duration;


	// sanity checks
	if (dt < 0)
		return false;


	out.xd = (end.x - start.x) / duration;
	out.x = start.x + dt * out.xd;
	out.xdd = 0;

	return true;
}


bool LinearSpline::getPoint(const double& user_dt, double& pos)
{
	Point out;
	LinearSpline::getPoint(user_dt, out);
	pos =  out.x;

	return true;
}

} //@namespace utils
} //@namespace dwl
