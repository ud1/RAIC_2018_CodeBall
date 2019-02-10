
#ifndef MY_UTILS_HPP
#define MY_UTILS_HPP

#ifdef MY_IMPLEMENTATION
#define GLOBAL_VAR
#else
#define GLOBAL_VAR extern
#endif // MY_IMPLEMENTATION

#if ENABLE_LOGGING
#include <iostream>
#include <fstream>
#include <iomanip>
//GLOBAL_VAR std::ofstream outputFile("out.txt");
#define LOGC(x)  std::cout << x << std::endl;
//#define LOGF(x) outputFile << "T " << g_world->getTick() << "-" << g_self->getTeammateIndex() << "| " << g_self->getRemainingCooldownTicks() << " " << g_self->getRemainingKnockdownTicks() << "| " << g_self->getX() << " " << g_self->getY() << " " << g_self->getAngle() << "| " << g_self->getSpeedX() << " " << g_self->getSpeedY() << "| " << x << std::endl;
//#define LOG(x) LOGC(g_world->getTickIndex() << " " << x);
#define LOG(x) LOGC(x);
#else
#define LOG(x)

#endif // ENABLE_LOGGING

extern long g_tick;


#define _USE_MATH_DEFINES

#include <cmath>
#include <ctime>
#include <vector>

constexpr double EPS = 1e-7;
constexpr float FEPS = 1e-2;

constexpr inline double sqr ( double v )
{
    return v*v;
}

constexpr double PI = 3.14159265358979323846;
constexpr double PI05 = PI * 0.5;

inline double normalizeAngle(double angle)
{
    while ( angle > PI ) {
        angle -= 2.0 * PI;
    }

    while ( angle < -PI ) {
        angle += 2.0 * PI;
    }

    return angle;
}

inline double atan2_approximation2( double y, double x )
{
    if ( x == 0.0 )
    {
        if ( y > 0.0 )
            return PI05;
        if ( y == 0.0 )
            return 0.0;
        return -PI05;
    }

    float atan;
    float z = y/x;
    if ( std::abs( z ) < 1.0 )
    {
        atan = z/(1.0 + 0.28*z*z);
        if ( x < 0.0 )
        {
            if ( y < 0.0 )
                return atan - PI;
            return atan + PI;
        }
    }
    else
    {
        atan = PI05 - z/(z*z + 0.28);
        if ( y < 0.0 )
            return atan - PI;
    }
    return atan;
}

/*inline double angleDiff(double angle1, double angle2)
{
	return std::abs(normalizeAngle(angle1 - angle2));
}*/

struct P {
    explicit P() {}

    constexpr explicit P ( double angle ) : x(cos(angle)), z(sin(angle)) {}
    constexpr explicit P ( double x, double y ) : x ( x ), z ( y ) {}

    constexpr double dist ( P const &o ) const {
        return sqrt ( sqr ( x - o.x ) + sqr ( z - o.z ) );
    }

    constexpr double dist2 ( P const &o ) const {
        return sqr ( x - o.x ) + sqr ( z - o.z );
    }

    constexpr double len2() const {
        return x*x + z*z;
    }

    constexpr double len() const {
        return sqrt ( x*x + z*z );
    }

    constexpr P norm() const {
        double l = len();
        if ( l == 0.0 )
            return P ( 1.0, 0.0 );
        return P ( x / l, z / l );
    }

    constexpr P conj() const {
        return P(z, -x);
    }

    constexpr P conjR() const {
        return P(-z, x);
    }

    constexpr P &operator += (const P &o)
    {
        x += o.x;
        z += o.z;
        return *this;
    }

    constexpr P &operator -= (const P &o)
    {
        x -= o.x;
        z -= o.z;
        return *this;
    }

    constexpr P &operator *= (double v)
    {
        x *= v;
        z *= v;
        return *this;
    }

    constexpr P &operator /= (double v)
    {
        double r = 1.0 / v;
        x *= r;
        z *= r;
        return *this;
    }

    constexpr bool operator == ( const P &o ) const {
        return ( std::abs ( x - o.x ) + std::abs ( z - o.z ) ) < EPS;
    }

    constexpr bool operator != ( const P &o ) const {
        return ( std::abs ( x - o.x ) + std::abs ( z - o.z ) ) >= EPS;
    }

    constexpr P rotate ( double angle ) const {
        double cs = cos ( angle );
        double sn = sin ( angle );
        return P ( x * cs - z * sn, x * sn + z * cs );
    }

    constexpr double getAngle () const {
        double absoluteAngleTo = atan2 (this->z, this->x );
        return absoluteAngleTo;
    }

    double getAngleFast () const {
        double absoluteAngleTo = atan2_approximation2(this->z, this->x );
        return absoluteAngleTo;
    }

    constexpr P rotate(const P &sinCos) const
    {
        return P(x * sinCos.x - z * sinCos.z, z * sinCos.x + x * sinCos.z);
    }

    template<int ind>
    double comp() {
        if (ind == 0)
            return x;
        return z;
    }

    double x, z;
};

inline P unrotate(const P &fromDir, const P& toDir)
{
    return P(toDir.x * fromDir.x + toDir.z * fromDir.z, toDir.z * fromDir.x - toDir.x * fromDir.z).norm();
}

inline P clampDir(const P &value, const P &maxValue)
{
    if (value.x >= maxValue.x)
        return value;

    if (value.z > 0.0)
        return maxValue;

    return P(maxValue.x, -maxValue.z);
}

inline bool isAngleLessThan(const P &value1, const P &value2)
{
    return value1.x >= value2.x;
}

inline double clamp ( double v, double vmin, double vmax )
{
    if ( v < vmin )
        return vmin;
    if ( v > vmax )
        return vmax;
    return v;
}

inline int clampI ( int v, int vmin, int vmax )
{
    if ( v < vmin )
        return vmin;
    if ( v > vmax )
        return vmax;
    return v;
}

inline float clampF ( float v, float vmin, float vmax )
{
    if ( v < vmin )
        return vmin;
    if ( v > vmax )
        return vmax;
    return v;
}

inline P clampP(const P &p, const P &minP, const P &maxP)
{
    return P(clamp(p.x, minP.x, maxP.x), clamp(p.z, minP.z, maxP.z));
}

#ifdef ENABLE_LOGGING
inline std::ostream &operator << ( std::ostream &str, const P&p )
{
    str << "(" << p.x << "," << p.z << ")";
    return str;
}
#endif

constexpr inline P operator - ( P p1, P p2 )
{
    return P ( p1.x - p2.x, p1.z - p2.z );
}

constexpr inline P operator + ( P p1, P p2 )
{
    return P ( p1.x + p2.x, p1.z + p2.z );
}

constexpr inline P operator * ( P p1, double v )
{
    return P ( p1.x * v, p1.z * v );
}

constexpr inline P operator / ( P p1, double v )
{
    return P ( p1.x / v, p1.z / v );
}

constexpr inline P operator / ( double v, P p1 )
{
    return P ( v / p1.x, v / p1.z );
}

constexpr inline P operator * ( P p1, P p2 )
{
    return P ( p1.x * p2.x, p1.z * p2.z );
}

constexpr inline P operator / ( P p1, P p2 )
{
    return P ( p1.x / p2.x, p1.z / p2.z );
}

constexpr inline double dot ( P p1, P p2 )
{
    return p1.x * p2.x + p1.z * p2.z;
}

constexpr inline double cross ( P p1, P p2 )
{
    return p1.x * p2.z - p1.z * p2.x;
}

inline P clampV(const P &v, double len)
{
    double l = v.len();
    if (l > len)
        return v * (len / l);

    return v;
}


constexpr inline P closestPointToSegment(const P &segP1, const P &segP2, const P &p)
{
    P r = segP2 - segP1;
    double rlen2 = r.len2();
    if (rlen2 == 0.0)
        return segP1;

    P t = p - segP1;

    double d = dot(r, t) / rlen2;
    if (d >= 1.0)
        return segP2;
    if (d <= 0.0)
        return segP1;

    P res = segP1 + r * d;
    return res;
}

constexpr inline bool checkSegsIntersect(const P &start1, const P &end1, const P &start2, const P &end2)
{
    P a = end1 - start1;
    P b = start2 - end2;
    P d = start2 - start1;

    double det = a.x * b.z - a.z * b.x;

    if (det == 0)
        return false;

    double r = (d.x * b.z - d.z * b.x) / det;
    double s = (a.x * d.z - a.z * d.x) / det;

    return !(r < 0 || r > 1 || s < 0 || s > 1);
}

template<class BidirIt, class UnaryPredicate>
BidirIt unstable_remove_if(BidirIt first, BidirIt last, UnaryPredicate p)
{
    while (1) {
        while ((first != last) && !p(*first)) {
            ++first;
        }
        if (first == last--) break;
        while ((first != last) && p(*last)) {
            --last;
        }
        if (first == last) break;
        *first++ = std::move(*last);
    }
    return first;
}

template<class T, class UnaryPredicate>
void filterVector(std::vector<T> &v, UnaryPredicate p)
{
    v.erase(unstable_remove_if(v.begin(), v.end(), p), v.end());
}

struct TimeMeasure
{
    TimeMeasure(double &total) : total(total) {
        c_start = std::clock();
    }

    ~TimeMeasure()
    {
        std::clock_t c_end = std::clock();
        total += 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
    }

    double &total;
    std::clock_t c_start;
};

#endif

