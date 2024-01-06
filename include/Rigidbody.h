#ifndef PHYSICS_RIGIDBODY_H
#define PHYSICS_RIGIDBODY_H

#include "ICollider.h"
#include "Vec3.h"
#include <memory>

namespace physics
{

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
    Vec3 angular;
};

class Rigidbody
{
public:
    Rigidbody(std::shared_ptr<ICollider> collider);

    const ICollider* Collider() const;
    const DoF& Displacement() const;
    const DoF& Velocity() const;

    sf::Shape& SFMLShape();

private:
    std::shared_ptr<ICollider> m_collider;
    DoF m_displacement;
    DoF m_velocity;
};

} // namespace physics

#endif // PHYSICS_RIGIDBODY_H
