#include "LineSegment.h"
#include <algorithm>

namespace physics
{

LineSegment::LineSegment(const Vec3& start, const Vec3& end)
    : m_start(start)
    , m_end(end)
    , m_tangent(end - start)
    // This cross product creates a perpendicular vector to the right side.
    , m_normal(m_tangent.Cross(Vec3{.z = 1}))
    , m_length(m_tangent.Magnitude())
{
    m_tangent.Normalize();
    m_normal.Normalize();
}

const Vec3& LineSegment::Start() const
{
    return m_start;
}

const Vec3& LineSegment::End() const
{
    return m_end;
}

const Vec3& LineSegment::Tangent() const
{
    return m_tangent;
}

const Vec3& LineSegment::Normal() const
{
    return m_normal;
}

float LineSegment::Length() const
{
    return m_length;
}

Vec3 LineSegment::FindClosestPointOnLine(const Vec3& external_point) const
{
    // Key idea: solve equation "projection = m_start + t * (m_end - m_start)" for t.
    const auto start_to_point = external_point - m_start;
    const auto start_to_end = m_end - m_start;

    // Reason for dividing it with squared magnitude instead of just magnitude:
    //   "(v - p1) * (p2 - p1) / |p2 - p1|" gives the 'distance' from p1 to the projection.
    //   However, the value 't' we want is a uniform scale between 0 and 1
    //   where 0 corresponds to 'm_start' and 1 corresponds to 'm_end'.
    //   Therefore "t = (v - p1) * (p2 - p1) / |p2 - p1|^2".
    float t = start_to_point.Dot(start_to_end) / start_to_end.SquaredMagnitude();

    // If t is not in range [0, 1], the projection lies outside of the line segment.
    // We can choose either m_start or m_end instead by clamping t.
    t = std::clamp(t, 0.0f, 1.0f);

    // Now we finally have the point on the line,
    // closest to the given external point!
    return m_start + t * start_to_end;
}

} // namespace physics

