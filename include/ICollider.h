#ifndef PHYSICS_I_COLLIDER_H
#define PHYSICS_I_COLLIDER_H

#include "SFML/Graphics/Shape.hpp"

namespace physics
{

enum class ColliderType
{
    ConvexPolygon,
    Circle
};

class ICollider
{
public:
    virtual ~ICollider() = default;
    virtual float BoundaryRadius() const = 0;
    virtual ColliderType Type() const = 0;
    virtual sf::Shape& SFMLShape() = 0;
};

} // namespace physics

#endif // PHYSICS_I_COLLIDER_H
