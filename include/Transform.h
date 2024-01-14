#ifndef PHYSICS_TRANSFORM_H
#define PHYSICS_TRANSFORM_H

#include "Vec3.h"
#include "LineSegment.h"
#include "Angle.h"

namespace physics
{

class Transform
{
public:
    const Vec3& Position() const;
    Radian Rotation() const;

    /**
     * @warning @p new_position should be on a 2D plane (i.e., new_position.z == 0)
     */
    void SetPosition(const Vec3& new_position);
    void SetRotation(Radian new_rotation);

    /**
     * @brief Helper functions for transforming coordinates
     *        between global and local coordinate systems.
     */
    Vec3 GlobalDirection(const Vec3& local_dir) const;
    Vec3 LocalDirection(const Vec3& global_dir) const;

    Vec3 GlobalPosition(const Vec3& local_pos) const;
    Vec3 LocalPosition(const Vec3& global_pos) const;

    LineSegment GlobalEdge(const LineSegment& local_edge) const;
    LineSegment LocalEdge(const LineSegment& global_edge) const;
    
private:
    Vec3 m_position = {};
    Radian m_rotation = 0.0f;
};

} // namespace physics


#endif // PHYSICS_TRANSFORM_H
