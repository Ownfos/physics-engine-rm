﻿#include "Rigidbody.h"
#include "Circle.h"
#include "ConvexPolygon.h"
#include <cassert>

namespace physics
{

Rigidbody::Rigidbody(
    std::shared_ptr<ICollider> collider,
    const MaterialProperties& material,
    float mass,
    float inertia
)
    : m_collider(collider), m_material(material)
{
    SetMass(mass);
    SetInertia(inertia);
}

ICollider* Rigidbody::Collider()
{
    return m_collider.get();
}

const ICollider* Rigidbody::Collider() const
{
    return m_collider.get();
}

const MaterialProperties& Rigidbody::Material() const
{
    return m_material;
}

const Vec3& Rigidbody::Position() const
{
    return m_displacement.linear;
}

const Vec3& Rigidbody::Rotation() const
{
    return m_displacement.angular;
}

const Vec3& Rigidbody::LinearVelocity() const
{
    return m_velocity.linear;
}

const Vec3& Rigidbody::AngularVelocity() const
{
    return m_velocity.angular;
}

float Rigidbody::InverseMass() const
{
    return m_inv_mass;
}

float Rigidbody::InverseInertia() const
{
    return m_inv_inertia;
}

bool Rigidbody::IsPointInside(const Vec3& global_pos) const
{
    // Since collider doesn't know about our transform,
    // we need to convert it to the corresponding local coordinate.
    return Collider()->IsPointInside(LocalPosition(global_pos));
}

Vec3 Rigidbody::GlobalPosition(const Vec3& local_pos) const
{
    // The offset of local point w.r.t. the
    // object's origin in global coordinate.
    auto rotated_offset = local_pos;
    rotated_offset.Rotate(Rotation().z);
    
    return Position() + rotated_offset;
}

Vec3 Rigidbody::LocalPosition(const Vec3& global_pos) const
{
    // The offset of global point w.r.t. the
    // object's origin in global coordinate (still rotated!).
    auto rotated_offset = global_pos - Position();
    rotated_offset.Rotate(-Rotation().z);
    
    return rotated_offset;
}

Vec3 Rigidbody::GlobalVelocity(const Vec3& local_pos) const
{
    return LinearVelocity() + AngularVelocity().Cross(local_pos);
}

void Rigidbody::SetPosition(const Vec3& new_position)
{
    assert(std::abs(new_position.z) < epsilon);

    m_displacement.linear = new_position;
}

void Rigidbody::MovePosition(const Vec3& offset)
{
    assert(std::abs(offset.z) < epsilon);

    m_displacement.linear += offset;
}

void Rigidbody::SetRotation(Radian new_rotation)
{
    m_displacement.angular.z = new_rotation;
}

/**
 * @brief Calculate the inverse of given real number.
 * @return (1 / value) if the value is nonzero.
 *         0 if the value is zero.
 */
float Inverse(float value)
{
    if (value < epsilon)
    {
        return 0.0f;
    }
    else
    {
        return 1.0f / value;
    }
}

void Rigidbody::SetMass(float new_mass)
{
    assert(new_mass >= 0.0f);

    m_inv_mass = Inverse(new_mass);
}

void Rigidbody::SetInertia(float new_inertia)
{
    assert(new_inertia >= 0.0f);

    m_inv_inertia = Inverse(new_inertia);
}

void Rigidbody::MakeObjectStatic()
{
    // Give infinite mass and inertia.
    m_inv_mass = 0.0f;
    m_inv_inertia = 0.0f;
}

bool Rigidbody::IsOutOfBoundaryRadius(const Rigidbody& other) const
{
    // Imagine that there are two circles with different radius.
    // The upper limit of distance where collision between them
    // can occur would be the sum of their radius.
    const auto max_collision_distance = Collider()->BoundaryRadius() + other.Collider()->BoundaryRadius();
    const auto squared_max_distance = max_collision_distance * max_collision_distance;

    // Now calculate the actual distance between their origin.
    const auto rel_pos = Position() - other.Position();
    const auto squared_distance = rel_pos.SquaredMagnitude();

    // If the actual distance is greater than the upper limit,
    // there is no chance of collision between two objects.
    return squared_distance > squared_max_distance;
}

std::optional<CollisionInfo> CircleToCircleCollisionCheck(Rigidbody& circle1, Rigidbody& circle2)
{
    const auto collider1 = dynamic_cast<const Circle*>(circle1.Collider());
    const auto collider2 = dynamic_cast<const Circle*>(circle2.Collider());

    // Precondition: both objects should have a circle shape as collider.
    assert(collider1 != nullptr);
    assert(collider2 != nullptr);

    // Record the objects under collision test.
    auto result = CollisionInfo{
        .object1 = &circle1,
        .object2 = &circle2
    };

    // The position of circle2 w.r.t. circle1's perspective.
    const auto center1 = circle1.Position();
    const auto center2 = circle2.Position();
    const auto circle2_rel_pos = center2 - center1;
    
    // The minimum distance required to separate two circles.
    // If they are just one step away from collision,
    // this would be the distance between their corresponding center.
    const auto radius1 = collider1->BoundaryRadius();
    const auto radius2 = collider2->BoundaryRadius();
    const auto min_separation_distance = radius1 + radius2;

    // Calculate the normal vector.
    if (circle2_rel_pos.IsZero())
    {
        // Since two circles are on the exactly same position,
        // the direction of the impulse doesn't matter.
        // 
        // Just arbitrarily choose (1, 0) as the normal vector
        // so that they push each other horizontally.
        result.normal = {1.0f, 0.0f};
    }
    else
    {
        // circle2 must move away from circle1.
        result.normal = circle2_rel_pos;
        result.normal.Normalize();
    }

    // The distance required to separate two circles.
    result.penetration_depth = min_separation_distance - circle2_rel_pos.Magnitude();

    // Case 1) collision happened!
    if (result.penetration_depth > 0.0f)
    {
        const auto contact_point = center1 + result.normal * radius1;
        result.contacts.push_back(contact_point);
        return result;
    }
    // Case 2) they were to far from each other...
    else
    {
        return {};
    }
}

std::optional<CollisionInfo> CircleToPolygonCollisionCheck(Rigidbody& circle, Rigidbody& polygon)
{
    // Key idea: there are two cases where collision occurs.
    // 1. circle's center is inside the polygon.
    // 2. circle's center is outside the polygon,
    //    but the distance is shorter than its radius.
    const auto collider1 = dynamic_cast<const Circle*>(circle.Collider());
    const auto collider2 = dynamic_cast<const ConvexPolygon*>(polygon.Collider());

    // Precondition: the objects should have circle and polygon collider respectively.
    assert(collider1 != nullptr);
    assert(collider2 != nullptr);

    // Record the objects under collision test.
    //
    // We assume that the circle is penetrating the polygon.
    // This means that the collision normal we are calculating
    // is the 'direction where the circle must move'.
    //
    // Since CollisionInfo treats 'normal' as the 'direction where object2 must move',
    // circle goes to object2, while polygon goes to object1.
    auto result = CollisionInfo{
        .object1 = &polygon,
        .object2 = &circle
    };

    // Case 1) check if the center of the circle is within the polygon.
    const auto is_circle_inside_poly = polygon.IsPointInside(circle.Position());

    // From now on, every calculation will be done under polygon's coordinate system.
    const auto circle_radius = collider1->BoundaryRadius();
    for (const auto& edge : collider2->Edges())
    {
        // The position of the circle's center w.r.t. the polygon.
        const auto circle_rel_pos = polygon.LocalPosition(circle.Position());

        // Case 2) check if the circle is close enough to the polygon's boundary.
        const auto closest_point = edge.FindClosestPointOnLine(circle_rel_pos);
        const auto edge_to_circle_center = circle_rel_pos - closest_point;
        const auto dist_from_edge = edge_to_circle_center.Dot(edge.Normal());
        const auto is_circle_touching_edge = dist_from_edge > 0.0f && dist_from_edge < circle_radius;

        // If either condition for collision is satisfied,
        // record the minimum penetration depth and the collision normal.
        if (is_circle_inside_poly || is_circle_touching_edge)
        {
            // Calculate the penetration depth based on the boundary of the circle.
            // Note that 'dist_from_edge' is a dot product w.r.t. the edge normal (headed outside!).
            // This makes dist_from_edge negative on points inside the polygon.
            const auto penetration_depth = circle_radius - dist_from_edge;

            // If this is the first edge that satisfies condition,
            // or the penetration depth was smaller than previous minimum
            if (result.contacts.empty() || result.penetration_depth > penetration_depth)
            {
                // Overwrite previous record with the new minimum penetration case.
                result.contacts.clear();
                result.contacts.push_back(polygon.GlobalPosition(closest_point));
                result.penetration_depth = penetration_depth;

                // Don't forget to convert local direction to global direction!
                result.normal = edge.Normal();
                result.normal.Rotate(polygon.Rotation().z);
            }
        }
    }

    if (result.contacts.empty())
    {
        return {};
    }
    else
    {
        return result;
    }
}

std::optional<CollisionInfo> PolygonToPolygonCollisionCheck(Rigidbody& polygon1, Rigidbody& polygon2)
{
    // TODO: implement
    return {};
}

std::optional<CollisionInfo> Rigidbody::CheckCollision(Rigidbody& other)
{
    // Return null if there is no chance of collision at all.
    //
    // This check allows skipping complex collision checks
    // between two objects too far from each other,
    // thus improving overall performance.
    if (IsOutOfBoundaryRadius(other))
    {
        return {};
    }

    // We only have two collider types (circle & polygon),
    // so there are four possible combinations.
    const bool is_this_circle = Collider()->Type() == ColliderType::Circle;
    const bool is_other_circle = other.Collider()->Type() == ColliderType::Circle;
    if (is_this_circle && is_other_circle)
    {
        return CircleToCircleCollisionCheck(*this, other);
    }
    else if (is_this_circle && !is_other_circle)
    {
        return CircleToPolygonCollisionCheck(*this, other);
    }
    else if (!is_this_circle && is_other_circle)
    {
        return CircleToPolygonCollisionCheck(other, *this);
    }
    else
    {
        return PolygonToPolygonCollisionCheck(*this, other);
    }
}

void Rigidbody::ApplyImpulse(const Vec3& rel_impact_pos, const Vec3& impulse, float delta_time)
{
    // J = F * Δt, assuming constant force.
    const auto force_over_time = impulse / delta_time;

    // τ = r x F
    m_acceleration.angular += rel_impact_pos.Cross(force_over_time) * m_inv_inertia;
    m_acceleration.linear += force_over_time * m_inv_mass;
}

void Rigidbody::Update(float delta_time)
{
    m_displacement.linear += m_velocity.linear * delta_time;
    m_displacement.angular += m_velocity.angular * delta_time;

    m_velocity.linear += m_acceleration.linear * delta_time;
    m_velocity.angular += m_acceleration.angular * delta_time;

    // Accumulated acceleration is valid only for a single time step.
    // Reset them to zero for next time step.
    m_acceleration.linear = {};
    m_acceleration.angular = {};

    // Synchonize SFML representation with physical state.
    // Note that SFML uses degree as unit, while our rotation is radian.
    auto& shape = m_collider->SFMLShape();
    shape.setPosition(m_displacement.linear.x, m_displacement.linear.y);
    shape.setRotation(rad2deg(m_displacement.angular.z));
}

} // namespace physics
