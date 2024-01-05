#ifndef PHYSICS_VEC3_H
#define PHYSICS_VEC3_H

#include <cmath>

namespace physics
{

constexpr float epsilon = 0.0000001f;

struct Vec3
{
    float x;
    float y;
    float z;

    inline float Dot(const Vec3& other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    inline Vec3 Cross(const Vec3& other) const
    {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.z
        };
    }

    inline float SquaredMagnitude() const
    {
        return x * x + y * y + z * z;
    }

    inline float Magnitude() const
    {
        return std::sqrt(SquaredMagnitude());
    }

    void Normalize()
    {
        if (auto magnitude = Magnitude(); magnitude < epsilon)
        {
            // Normalizing a zero vector is impossible.
            // Just keep it as a zero vector.
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        else
        {
            x /= magnitude;
            y /= magnitude;
            z /= magnitude;
        }
    }
};

inline Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z
    };
}

inline Vec3 operator-(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z
    };
}

inline Vec3 operator/(const Vec3& v, float scale)
{
    return {
        v.x / scale,
        v.y / scale,
        v.z / scale
    };
}

inline Vec3 operator*(const Vec3& v, float scale)
{
    return {
        v.x * scale,
        v.y * scale,
        v.z * scale
    };
}

inline Vec3 operator*(float scale, const Vec3& v)
{
    return scale * v;
}

} // namespace physics

#endif // PHYSICS_VEC3_H
