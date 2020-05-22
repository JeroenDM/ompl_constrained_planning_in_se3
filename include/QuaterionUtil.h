#ifndef OMPL_DEMO_QUATERNION_UTIL_
#define OMPL_DEMO_QUATERNION_UTIL_

#include <cmath>
#include <limits>
#include <iostream>
#include <cassert>

namespace ompl
{
namespace util
{

static const double MAX_QUATERNION_NORM_ERROR = 1e-9;

/* \Brief Quaternion data structure
*
* Operations on quaternions are not defined as members.
* This is in line with some of the
* other design decisions in ompl.
* The print function is an exception as most ompl objects
* have a print function as a member.
*
* The goal of using this structure is to reuse already implemented
* functions from ompl where often the quaternion is given as
* an object that has data members x, y, z and w.
*
* TODO: This implementation choice means that we probably need to
* create a lot of Quaternion objects from raw array values during
* runtime. I'm quit sure there are better choices, but I don't know how.
*/
struct Quaternion
{
    double x, y, z, w;

    Quaternion(double xi=0.0, double yi=0.0, double zi=0.0, double wi=0.0)
      :  x(xi), y(yi), z(zi), w(wi) {}

    /* \Brief Construct from raw array */
    Quaternion(double* values)
    {
        x = *values++;
        y = *values++;
        z = *values++;
        w = *values++;
    }

    void print(std::ostream &out) const
    {
        out << "Quaternion: [ ";
        out << x << ", " << y << ", " << z << ", " << w;
        out << "]" << std::endl;
    }
};

void setIdentityQuaternion(Quaternion& q)
{
    q.x = q.y = q.z = 0.0;
    q.w = 1;
}

double quaternionNormSquared(const Quaternion& q)
{
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

/* \Brief quaternion norm including close to one check.
* As is done in the ompl SO3 class, the norm calculation
* check if the result is close to 1 and returns 1 if this
* is the case.
*/
double quaternionNorm(const Quaternion& q)
{
    double ns = quaternionNormSquared(q);
    if (fabs(ns - 1.0) > std::numeric_limits<double>::epsilon())
        return std::sqrt(ns);
    else
        return 1.0;
    
}

void normalizeQuaternion(Quaternion& q)
{
    double nrmsq = quaternionNormSquared(q);
    double error = std::abs(1.0 - nrmsq);
    const double epsilon = 2.107342e-08;
    if (error < epsilon)
    {
        double scale = 2.0 / (1.0 + nrmsq);
        q.x *= scale;
        q.y *= scale;
        q.z *= scale;
        q.w *= scale;
    }
    else
    {
        if (nrmsq < 1e-6)
            setIdentityQuaternion(q);
        else
        {
            double scale = 1.0 / std::sqrt(nrmsq);
            q.x *= scale;
            q.y *= scale;
            q.z *= scale;
            q.w *= scale;
        }
    }
}

Quaternion operator * (const Quaternion q0, const Quaternion q1)
{
    Quaternion q;
    q.x = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y;
    q.y = q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z;
    q.z = q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x;
    q.w = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z;
    return q;
}

static inline double arcLength(const Quaternion from, const Quaternion to)
{
    double dq = fabs(from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w);
    if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
        return 0.0;
    return acos(dq);
}

static inline void computeAxisAngle(Quaternion &q, double ax, double ay, double az, double angle)
{
    double norm = std::sqrt(ax * ax + ay * ay + az * az);
    // if the input vector a has zero norm, return identity quaterion
    if (norm < MAX_QUATERNION_NORM_ERROR)
        setIdentityQuaternion(q);
    else
    {
        double half_angle = angle / 2.0;
        double s = sin(half_angle) / norm;
        q.x = s * ax;
        q.y = s * ay;
        q.z = s * az;
        q.w = cos(half_angle);
    }
}

/* \Brief Fancy quaternion interpolation
* 
* For the most part copied from ompl/base SO3StateSpace.cpp, which is based on:
* Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
void interpolateQuaternions(const Quaternion &from, const Quaternion &to, const double t, Quaternion &result)
{
    //assert((fabs(quaternionNorm(from)) - 1.0) < MAX_QUATERNION_NORM_ERROR);
    //assert((fabs(quaternionNorm(to)) - 1.0) < MAX_QUATERNION_NORM_ERROR);

    double theta = arcLength(from, to);
    if (theta > std::numeric_limits<double>::epsilon())
    {
        double d = 1.0 / sin(theta);
        double s0 = sin((1.0 - t) * theta);
        double s1 = sin(t * theta);
        double dq = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
        if (dq < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
            s1 = -s1;

        result.x = (from.x * s0 + to.x * s1) * d;
        result.y = (from.y * s0 + to.y * s1) * d;
        result.z = (from.z * s0 + to.z * s1) * d;
        result.w = (from.w * s0 + to.w * s1) * d;
    }
    else
    {
        // from and two represent the same orientation
        result = from;
    }
}

} // namespace util
} // namespace ompl
#endif