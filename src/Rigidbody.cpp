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
    return Collider()->Transform().Position();
    // return m_displacement.linear;
}

Radian Rigidbody::Rotation() const
{
    return Collider()->Transform().Rotation();
    // return m_displacement.angular;
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

const physics::Transform& Rigidbody::Transform() const
{
    return Collider()->Transform();
}

physics::Transform& Rigidbody::Transform()
{
    return Collider()->Transform();
}

bool Rigidbody::IsPointInside(const Vec3& global_pos) const
{
    // Since collider doesn't know about our transform,
    // we need to convert it to the corresponding local coordinate.
    return Collider()->IsPointInside(Collider()->Transform().LocalPosition(global_pos));
}

Vec3 Rigidbody::GlobalVelocity(const Vec3& local_pos) const
{
    return LinearVelocity() + AngularVelocity().Cross(local_pos);
}

void Rigidbody::SetPosition(const Vec3& new_position)
{
    Collider()->Transform().SetPosition(new_position);
}

void Rigidbody::MovePosition(const Vec3& offset)
{
    Collider()->Transform().SetPosition(Position() + offset);
}

void Rigidbody::SetRotation(Radian new_rotation)
{
    Collider()->Transform().SetRotation(new_rotation);
    // m_displacement.angular.z = new_rotation;
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

std::optional<CollisionPair> Rigidbody::CheckCollision(Rigidbody& other)
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

    auto result = Collider()->CheckCollision(other.Collider());
    if (result)
    {
        return CollisionPair{
            .object1 = this,
            .object2 = &other,
            .contacts = result->contacts,
            .normal = result->normal,
            .penetration_depth = result->penetration_depth
        };
    }
    else
    {
        return {};
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
    SetPosition(Position() + m_velocity.linear * delta_time);
    SetRotation(Rotation() + m_velocity.angular.z * delta_time);
    // m_displacement.linear += m_velocity.linear * delta_time;
    // m_displacement.angular += m_velocity.angular * delta_time;

    m_velocity.linear += m_acceleration.linear * delta_time;
    m_velocity.angular += m_acceleration.angular * delta_time;

    // Accumulated acceleration is valid only for a single time step.
    // Reset them to zero for next time step.
    m_acceleration.linear = {};
    m_acceleration.angular = {};

    // Synchonize SFML representation with physical state.
    // Note that SFML uses degree as unit, while our rotation is radian.
    auto& shape = m_collider->SFMLShape();

    auto [x, y, _] = Position();
    shape.setPosition(x, y);
    shape.setRotation(rad2deg(Rotation()));
}

} // namespace physics
