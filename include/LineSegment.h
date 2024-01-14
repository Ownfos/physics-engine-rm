#ifndef PHYSICS_LINE_SEGMENT_H
#define PHYSICS_LINE_SEGMENT_H

#include "Vec3.h"

namespace physics
{

/**
 * @brief LineSegment represents a finite line with start and end point.
 * 
 * @note In case of a default-constructed instance,
 *       every method will simply return a zero vector.
 * 
 * @note The coordinate system used to represent
 *       boundary points can be either local or global.
 *       Use Transform::GlobalEdge() and Transform::LocalEdge()
 *       for conversion between coordinate systems.
 */
class LineSegment
{
public:
    LineSegment() = default;
    LineSegment(const Vec3& start, const Vec3& end);

    const Vec3& Start() const;
    const Vec3& End() const;

    /**
     * @return Normalized vector with direction from Start() to End().
     */
    const Vec3& Tangent() const;

    /**
     * @return Normalized vector perpendicular with Tangent().
     * @note The direction is headed to the right.
     * @note ex) normal vector for tangent vector (0, 1) is (1, 0).
     */
    const Vec3& Normal() const;

    /**
     * @return Distance between Start() and End().
     */
    float Length() const;

    /**
     * @return The point on this line segment with
     *         shortest distance from given point.
     * 
     * @note The result differs from projection on an infinite line
     *       iff the projection lies outside of this line segment.
     *       Otherwise, vector from result to @p external_point
     *       is perpendicular to this line segment.
     */
    Vec3 FindClosestPointOnLine(const Vec3& external_point) const;

    /**
     * @brief Find a subrange of this line segment that resides within
     *        the range defined by @p reference.
     * 
     * @param reference_edge A line segment that defines the clipping region.
     * 
     * @warning @p reference should use global coordinate system.
     * 
     * @note 'clipping region' refers to an infinite rectangle
     *        between start end end point of the @p reference.
     */
    LineSegment Clip(const LineSegment& reference) const;

private:
    Vec3 m_start;
    Vec3 m_end;
    Vec3 m_tangent;
    Vec3 m_normal;
    float m_length;
};

} // namespace physics

#endif // PHYSICS_LINE_SEGMENT_H
