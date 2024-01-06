#include "Rigidbody.h"

namespace physics
{

Rigidbody::Rigidbody(std::shared_ptr<ICollider> collider)
    : m_collider(collider)
{}

const ICollider* Rigidbody::Collider() const
{
    return m_collider.get();
}

const DoF& Rigidbody::Displacement() const
{
    return m_displacement;
}

const DoF& Rigidbody::Velocity() const
{
    return m_velocity;
}

sf::Shape& Rigidbody::SFMLShape()
{
    auto& shape = m_collider->SFMLShape();
    shape.setPosition(m_displacement.linear.x, m_displacement.linear.y);
    shape.setRotation(m_displacement.angular.z);

    return shape;
}

} // namespace physics
