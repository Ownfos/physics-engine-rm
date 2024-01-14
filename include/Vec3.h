#ifndef PHYSICS_VEC3_H
#define PHYSICS_VEC3_H

#include "Angle.h"
#include <cmath>

namespace physics
{

constexpr float epsilon = 0.0000001f;

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3& operator+=(const Vec3& other);
    Vec3& operator-=(const Vec3& other);
    Vec3& operator/=(float scale);
    Vec3& operator*=(float scale);

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator-() const;
    Vec3 operator/(float scale) const;
    Vec3 operator*(float scale) const;
    friend Vec3 operator*(float scale, const Vec3& v);

    float Dot(const Vec3& other) const;
    Vec3 Cross(const Vec3& other) const;

    float SquaredMagnitude() const;
    float Magnitude() const;

    bool IsZero() const;

    /**
     * @note We cannot normalize a zero vector!
     *       In such case, the vector stays the same.
     */
    void Normalize();

    void Rotate(Radian angle);
    Vec3 Rotated(Radian angle) const;

    Vec3 Projection(const Vec3& normalized_dir) const;
};

} // namespace physics

#endif // PHYSICS_VEC3_H
