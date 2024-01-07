#ifndef PHYSICS_VEC3_H
#define PHYSICS_VEC3_H

#include <cmath>

namespace physics
{

constexpr float epsilon = 0.0000001f;

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

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

    inline bool IsZero() const
    {
        return SquaredMagnitude() < epsilon;
    }

    void Normalize()
    {
        // Note: We cannot normalize a zero vector!
        //       This if statement prevents divide-by-zero.
        if (!IsZero())
        {
            const auto m = Magnitude();
            x /= m;
            y /= m;
            z /= m;
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
