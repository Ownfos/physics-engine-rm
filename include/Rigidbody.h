#ifndef PHYSICS_RIGIDBODY_H
#define PHYSICS_RIGIDBODY_H

#include "ICollider.h"
#include "CollisionInfo.h"
#include "Vec3.h"
#include <memory>
#include <optional>

namespace physics
{

// Type aliases used to distinguish angle units.
using Degree = float;
using Radian = float;

// "DoF" Stands for "Degree of Freedom".
// It represents displacement and velocity of a rigidbody.
struct DoF
{
    Vec3 linear;

    // Only uses the z-axis.
    // Treating angular disposition & velocity as a vector
    // allows easier calculation of following things:
    // - global linear velocity of a local point inside a rigidbody
    // - torque from a linear impulse on a local point inside a rigidbody
    //
    // Note: unit is radian, not degrees.
    Vec3 angular;
};

class Rigidbody
{
public:
    Rigidbody(std::shared_ptr<ICollider> collider);

    const ICollider* Collider() const;
    const Vec3& Position() const;
    const Vec3& Rotation() const;
    const Vec3& LinearVelocity() const;
    const Vec3& AngularVelocity() const;

    void SetPosition(const Vec3& new_position);
    void SetRotation(Radian new_rotation);

    sf::Shape& SFMLShape();

    // Returns true if the distance between these objects
    // are larger than the sum of their boundary radius,
    // which means that they are impossible to collide.
    bool IsOutOfBoundaryRadius(const Rigidbody& other) const;

    // Return the collision information if the two objects collided.
    // Note: This function is non-const because we need to apply impulse
    //       using object pointers stored in CollisionInfo,
    //       and applying impulse involves state change in displacement and velocity.
    std::optional<CollisionInfo> CheckCollision(Rigidbody& other);

private:
    std::shared_ptr<ICollider> m_collider;
    DoF m_displacement;
    DoF m_velocity;
};

} // namespace physics

#endif // PHYSICS_RIGIDBODY_H
