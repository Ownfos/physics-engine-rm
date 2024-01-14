#include "Vec3.h"

namespace physics
{

Vec3& Vec3::operator+=(const Vec3& other)
{
    x += other.x;
    y += other.y;
    z += other.z;

    return *this;
}

Vec3& Vec3::operator-=(const Vec3& other)
{
    x -= other.x;
    y -= other.y;
    z -= other.z;

    return *this;
}

Vec3& Vec3::operator/=(float scale)
{
    x /= scale;
    y /= scale;
    z /= scale;
    
    return *this;
}

Vec3& Vec3::operator*=(float scale)
{
    x *= scale;
    y *= scale;
    z *= scale;
    
    return *this;
}

Vec3 Vec3::operator+(const Vec3& other) const
{
    return {
        x + other.x,
        y + other.y,
        z + other.z
    };
}

Vec3 Vec3::operator-(const Vec3& other) const
{
    return {
        x - other.x,
        y - other.y,
        z - other.z
    };
}

Vec3 Vec3::operator-() const
{
    return {
        -x,
        -y,
        -z
    };
}

Vec3 Vec3::operator/(float scale) const
{
    return {
        x / scale,
        y / scale,
        z / scale
    };
}

Vec3 Vec3::operator*(float scale) const
{
    return {
        x * scale,
        y * scale,
        z * scale
    };
}

Vec3 operator*(float scale, const Vec3& v)
{
    return v * scale;
}

float Vec3::Dot(const Vec3& other) const
{
    return x * other.x + y * other.y + z * other.z;
}

Vec3 Vec3::Cross(const Vec3& other) const
{
    return {
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    };
}

float Vec3::SquaredMagnitude() const
{
    return x * x + y * y + z * z;
}

float Vec3::Magnitude() const
{
    return std::sqrt(SquaredMagnitude());
}

bool Vec3::IsZero() const
{
    return SquaredMagnitude() < epsilon;
}

void Vec3::Normalize()
{
    // This if statement prevents divide-by-zero.
    if (!IsZero())
    {
        const auto m = Magnitude();
        x /= m;
        y /= m;
        z /= m;
    }
}

void Vec3::Rotate(Radian angle)
{
    const auto cos = std::cos(angle);
    const auto sin = std::sin(angle);

    // Warning: directly assigning new value corrupts the calculation of y!
    const auto new_x = x * cos - y * sin;
    const auto new_y = y * cos + x * sin;

    x = new_x;
    y = new_y;
}

Vec3 Vec3::Rotated(Radian angle) const
{
    auto result = *this;
    result.Rotate(angle);
    
    return result;
}

Vec3 Vec3::Projection(const Vec3& normalized_dir) const
{
    return normalized_dir * Dot(normalized_dir);
}

} // namespace physics
