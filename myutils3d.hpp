#ifndef MYUTILS3D_HPP
#define MYUTILS3D_HPP

#include "myutils.hpp"


typedef double F;





inline F maxF(F f1, F f2)
{
    return std::max(f1, f2);
}













struct P3 {
    explicit P3() {}
    explicit P3(F f) : x(f), y(f), z(f) {}

    constexpr explicit P3 ( F x, F y, F z ) : x ( x ), y ( y ), z( z ) {}

    constexpr F dist ( P3 const &o ) const {
        return sqrt ( sqr ( x - o.x ) + sqr ( y - o.y ) + sqr ( z - o.z ) );
    }

    constexpr F dist2 ( P3 const &o ) const {
        return sqr ( x - o.x ) + sqr ( y - o.y ) + sqr ( z - o.z );
    }

    constexpr F len2() const {
        return x*x + y*y + z*z;
    }

    constexpr F len() const {
        return sqrt ( x*x + y*y + z*z);
    }

    constexpr P3 norm() const {
        F l = len();
        if ( l == 0.0 )
            return P3 ( 1.0, 0.0, 0.0);
        return P3 ( x / l, y / l, z / l );
    }

    constexpr P3 &operator += (const P3 &o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    constexpr P3 &operator -= (const P3 &o)
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    constexpr P3 &operator *= (F v)
    {
        x *= v;
        y *= v;
        z *= v;
        return *this;
    }

    constexpr P3 &operator /= (F v)
    {
        F r = 1.0 / v;
        x *= r;
        y *= r;
        z *= r;
        return *this;
    }

    constexpr bool operator == ( const P3 &o ) const {
        return ( std::abs ( x - o.x ) + std::abs ( y - o.y ) + std::abs ( z - o.z ) ) < EPS;
    }

    constexpr bool operator != ( const P3 &o ) const {
        return ( std::abs ( x - o.x ) + std::abs ( y - o.y ) + std::abs ( z - o.z ) ) >= EPS;
    }
    
    template<int ind>
    F comp() {
        if (ind == 0)
            return x;
        if (ind == 1)
            return y;
        return z;
    }

    P xz() const
    {
        return P(x, z);
    }

    F x, y, z;
};

inline P3 xz(const P &p, F y)
{
    return P3(p.x, y, p.z);
}

constexpr inline P3 operator - ( P3 p1, P3 p2 )
{
    return P3 ( p1.x - p2.x, p1.y - p2.y, p1.z - p2.z );
}

constexpr inline P3 operator + ( P3 p1, P3 p2 )
{
    return P3 ( p1.x + p2.x, p1.y + p2.y, p1.z + p2.z );
}

constexpr inline P3 operator * ( P3 p1, F v )
{
    return P3 ( p1.x * v, p1.y * v, p1.z * v );
}

constexpr inline P3 operator * ( F v, P3 p1 )
{
    return P3 ( p1.x * v, p1.y * v, p1.z * v );
}

constexpr inline P3 operator / ( P3 p1, F v )
{
    return P3 ( p1.x / v, p1.y / v, p1.z / v );
}

constexpr inline P3 operator / ( F v, P3 p1 )
{
    return P3 ( v / p1.x, v / p1.y, v / p1.z );
}

constexpr inline P3 operator * ( P3 p1, P3 p2 )
{
    return P3 ( p1.x * p2.x, p1.y * p2.y,  p1.z * p2.z );
}

constexpr inline P3 operator / ( P3 p1, P3 p2 )
{
    return P3 ( p1.x / p2.x, p1.y / p2.y, p1.z / p2.z );
}

constexpr inline F dot ( P3 p1, P3 p2 )
{
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

constexpr inline P3 cross ( P3 p1, P3 p2 )
{
    return P3(
            p1.y * p2.z - p1.z * p2.y,
            p1.z * p2.x - p1.x * p2.z,
            p1.x * p2.y - p1.y * p2.x
            );
}

inline F length(const P3 p)
{
    return p.len();
}

inline P3 clampV(const P3 &v, F len)
{
    F l = v.len();
    if (l > len)
        return v * (len / l);

    return v;
}






struct Rect {
    P3 pos;
    P3 t1, t2;
};

inline P3 closestPoint(const Rect &rect, const P3 &p)
{
    P3 rp = p - rect.pos;
    float t1 = dot(rect.t1, rp) / dot(rect.t1, rect.t1);
    float t2 = dot(rect.t2, rp) / dot(rect.t2, rect.t2);

    t1 = clampF(t1, -1, 1);
    t2 = clampF(t2, -1, 1);

    return rect.pos + rect.t1 * t1 + rect.t2 * t2;
}


#ifdef ENABLE_LOGGING
inline std::ostream &operator << ( std::ostream &str, const P3 &p )
{
    str << "(" << p.x << "," << p.y << "," << p.z << ")";
    return str;
}
#endif


inline F random(F vmin, F vmax)
{
    return rand() / (F) RAND_MAX * (vmax - vmin) + vmin;
}

#endif // MYUTILS3D_HPP
