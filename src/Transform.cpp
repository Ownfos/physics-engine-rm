#include "Transform.h"
#include <cassert>

namespace physics
{

const Vec3& Transform::Position() const
{
    return m_position;
}

Radian Transform::Rotation() const
{
    return m_rotation;
}

void Transform::SetPosition(const Vec3& new_position)
{
    assert(std::abs(new_position.z) < epsilon);

    m_position = new_position;
}

void Transform::SetRotation(Radian new_rotation)
{
    m_rotation = new_rotation;
}

Vec3 Transform::GlobalDirection(const Vec3& local_dir) const
{
    return local_dir.Rotated(Rotation());
}

Vec3 Transform::LocalDirection(const Vec3& global_dir) const
{
    return global_dir.Rotated(-Rotation());
}

Vec3 Transform::GlobalPosition(const Vec3& local_pos) const
{
    return Position() + GlobalDirection(local_pos);
}

Vec3 Transform::LocalPosition(const Vec3& global_pos) const
{
    return LocalDirection(global_pos - Position());
}

LineSegment Transform::GlobalEdge(const LineSegment& global_edge) const
{
    return {
        GlobalPosition(global_edge.Start()),
        GlobalPosition(global_edge.End())
    };
}

LineSegment Transform::LocalEdge(const LineSegment& local_edge) const
{
    return {
        LocalPosition(local_edge.Start()),
        LocalPosition(local_edge.End())
    };
}

} // namespace physics
