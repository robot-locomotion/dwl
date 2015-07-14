#include <utils/SplineInterpolation.h>


namespace dwl
{


namespace math
{

Spline::Spline() : initial_time_(0.), duration_(0.)
{

}


Spline::Spline(double initial_time, double duration, const Point& start_p, const Point& end_p) :
	initial_time_(initial_time), duration_(duration), start_(start_p), end_(end_p)
{
	if (duration <= 0)
		throw std::invalid_argument("Cannot create a Spliner with zero or negative duration");
}


Spline::~Spline()
{

}


void Spline::setBoundary(double initial_time, double duration, const Point& start_p, const Point& end_p)
{
	initial_time_ = initial_time;
	duration_ = duration;
	start_ = start_p;
	end_ = end_p;
}


void Spline::setBoundary(double initial_time, double duration, const double& start_p, const double& end_p)
{
	initial_time_ = initial_time;
	duration_ = duration;

	start_.x = start_p;
	start_.xd = 0.0;
	start_.xdd = 0.0;

	end_.x = end_p;
	end_.xd = 0.0;
	end_.xdd = 0.0;
}


CubicSpline::CubicSpline(double initial_time, double duration, const Point& start, const Point& end) :
	Spline(initial_time, duration, start, end)
{

}


CubicSpline::CubicSpline()
{

}


CubicSpline::~CubicSpline()
{

}


bool CubicSpline::getPoint(const double& current_time, Point& out)
{
    double dt = current_time - initial_time_;
    if (dt > duration_)
        dt = duration_;

    // sanity checks
    if (dt < 0)
        return false;

    double a0, a1, a2, a3;
    double T1, T2, T3;
    double dt2, dt3;

    // powers of the duration
    T1 = duration_;
    T2 = duration_ * T1;
    T3 = duration_ * T2;

    // powers of dt
    dt2 = dt * dt;
    dt3 = dt * dt2;

    // spline coefficients
    a0 = start_.x;
    a1 = start_.xd;
    a2 = -((3 * a0) - (3 * end_.x) + (2 * T1 * a1) + (T1 * end_.xd)) / T2;
    a3 = ((2 * a0) - (2 * end_.x) + T1 * (a1 + end_.xd)) / T3;

    // interpolated point
    out.x = a0 + a1 * dt + a2 * dt2 + a3 * dt3;
    out.xd = a1 + 2 * a2 * dt + 3 * a3 * dt2;
    out.xdd = 2 * a2 + 6 * a3 * dt;

    return true;
}


bool CubicSpline::getPoint(const double& current_time, double& pos)
{
	Point out;
	CubicSpline::getPoint(current_time, out);
	pos = out.x;

	return true;
}


FifthOrderPolySpline::FifthOrderPolySpline()
{

}


FifthOrderPolySpline::FifthOrderPolySpline(double initial_time, double duration, const Point& start, const Point& end) :
	Spline(initial_time, duration, start, end)
{

}

FifthOrderPolySpline::~FifthOrderPolySpline()
{

}


bool FifthOrderPolySpline::getPoint(const double& current_time, Point& out)
{
    double dt = current_time - initial_time_;
    if (dt > duration_)
        dt = duration_;

    // sanity checks
    if (dt < 0)
        return false;

    double a0, a1, a2, a3, a4, a5;
    double T1, T2, T3, T4, T5;
    double dt2, dt3, dt4, dt5;

    // powers of duration
    T1 = duration_;
    T2 = duration_ * duration_;
    T3 = duration_ * T2;
    T4 = duration_ * T3;
    T5 = duration_ * T4;

    dt2 = dt * dt;
    dt3 = dt * dt2;
    dt4 = dt * dt3;
    dt5 = dt * dt4;

    // spline coefficients
    a0 = start_.x;
    a1 = start_.xd;
    a2 = start_.xdd / 2;
    a3 = (-20 * a0 + 20 * end_.x + T1 * (-3 * a2 / 2 * T1 + end_.xdd * T1 - 12 * a1 - 8 * end_.xd)) / (2 * T3);
    a4 = (30 * a0 - 30 * end_.x + T1 * (3 * start_.xdd * T1 - 2 * end_.xdd * T1 + 16 * start_.xd + 14 * end_.xd)) / (2 * T4);
    a5 = -(12 * start_.x - 12 * end_.x + T1 * (start_.xdd * T1 - end_.xdd * T1 + 6 * (start_.xd + end_.xd))) / (2 * T5);

    out.x = a0 + a1 * dt + a2 * dt2 + a3 * dt3 + a4 * dt4 + a5 * dt5;
    out.xd = a1 + 2 * a2 * dt + 3 * a3 * dt2 + 4 * a4 * dt3 + 5 * a5 * dt4;
    out.xdd = a2 + 6 * a3 * dt + 12 * a4 * dt2 + 20 * a5 * dt3;

    return true;
}


bool FifthOrderPolySpline::getPoint(const double& current_time, double& pos)
{
	Point out;
	FifthOrderPolySpline::getPoint(current_time, out);
	pos = out.x;

	return true;
}


LinearSpline::LinearSpline()
{

}


LinearSpline::LinearSpline(double initial_time, double duration, const Point& start, const Point& end) :
	Spline(initial_time, duration, start, end)
{

}


LinearSpline::~LinearSpline()
{

}


bool LinearSpline::getPoint(const double& current_time, Point& out)
{
	double dt = current_time - initial_time_;
	if (dt > duration_)
		dt = duration_;

	// sanity checks
	if (dt < 0)
		return false;


	out.xd = (end_.x - start_.x) / duration_;
	out.x = start_.x + dt * out.xd;
	out.xdd = 0;

	return true;
}


bool LinearSpline::getPoint(const double& current_time, double& pos)
{
	Point out;
	LinearSpline::getPoint(current_time, out);
	pos =  out.x;

	return true;
}

} //@namespace utils
} //@namespace dwl
