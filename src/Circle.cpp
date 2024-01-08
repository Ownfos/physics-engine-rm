#include "Circle.h"

namespace physics
{

Circle::Circle(float radius)
    : m_shape(radius)
{
    // sf::CircleShape has origin on the corner,
    // so we should adjust it to the origin.
    m_shape.setOrigin({radius, radius});
}

float Circle::BoundaryRadius() const
{
    return m_shape.getRadius();
}

ColliderType Circle::Type() const
{
    return ColliderType::Circle;
}

sf::Shape& Circle::SFMLShape()
{
    return m_shape;
}

const sf::Shape& Circle::SFMLShape() const
{
    return m_shape;
}

} // namespace physics
